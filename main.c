#include <jo/jo.h>

/* used only for free fly */
#include "mathex.h"

/* Simulation stuff */
#define CLOTH_SIZE_W (25)
#define CLOTH_SIZE_H (16)
#define CLOTH_SPACING JO_FIXED_2

/** @brief Phys point
 */
typedef struct {
    int PosIndex;
    jo_pos3D_fixed PrevPos;
    bool Locked;
} PhysPoint;

typedef struct
{
    PhysPoint * First;
    PhysPoint * Second;
    jo_fixed Length;
} Segment;

/* Mesh points */
static POINT MeshPoints[CLOTH_SIZE_W * CLOTH_SIZE_H];

/* All points of cloth */
static PhysPoint Points[CLOTH_SIZE_W * CLOTH_SIZE_H];

/* All connecting segments */
static Segment Segments[(CLOTH_SIZE_W * (CLOTH_SIZE_H - 1)) + ((CLOTH_SIZE_W - 1) * CLOTH_SIZE_H)];

/* Gravity direction */
static jo_vector_fixed Gravity;

/* Cloth mesh */
static jo_3d_mesh Cloth;

/* The box */
#define BOX_SIZE (8)
static jo_pos3D_fixed BoxPosition = { -(BOX_SIZE << 17), 0, BOX_SIZE << 16 };
static jo_vertice BoxVertices[] = JO_3D_CUBE_VERTICES(BOX_SIZE);
static jo_3d_quad BoxQuads[6];
static int BoxTextureIndex = 0;

/* Camera stuff */
/* Speed of moving */
#define MOVE_SPEED JO_FIXED_1_DIV_2

/* Speed of turning */
#define TURN_SPEED (1319)

/* Camera location and orientation */
static ExCoordinateSystem Camera = { { ((CLOTH_SIZE_W >> 1) * CLOTH_SPACING), -(JO_FIXED_32 << 1), JO_FIXED_32 }, {{ JO_FIXED_1, 0, 0 }}, {{ 0, 0, -JO_FIXED_1 }} };

/* RGB0 plane palette */
static jo_palette Rgb0Palette;

/* Is player controlling cloth or camera */
static int ControlMode = 0;

/* Is cloth going to drag on cube */
static bool CubeIsSticky = 0;

/* Current cloth texture */
static int ClothTexture = 0;

/* maximum number of cloth textures */
#define MAX_CLOTH_TEXTURES (4)

/** @brief Move camera forward and backward
 *  @param value Speed of movement
 */
void DoThrust(jo_fixed value)
{
    jo_vector_fixed cross = ExVectorCross(&Camera.AxisX, &Camera.AxisY);
    ExVectorNormal(&cross);

    jo_vector_fixed thrustVector;
    jo_vector_fixed_muls(&cross, value, &thrustVector);
    ExTranslatePoint(&Camera.Origin, &thrustVector);

    Camera.Origin.z = MAX(Camera.Origin.z, JO_FIXED_2);
}

/** @brief Rotate camera around its Up vector
 *  @param value Speed of movement
 */
void DoYaw(jo_fixed value)
{
    Camera.AxisX = ExRotateVectorAroundAxis(&Camera.AxisX, value, &Camera.AxisY);
}

/** @brief Rotate camera around its side vector
 *  @param value Speed of movement
 */
void DoPitch(jo_fixed value)
{
    Camera.AxisY = ExRotateVectorAroundAxis(&Camera.AxisY, value, &Camera.AxisX);
}

/** @brief Rotate camera around its direction vector
 *  @param value Speed of movement
 */
void DoRoll(jo_fixed value)
{
    jo_vector_fixed cross = ExVectorCross(&Camera.AxisX, &Camera.AxisY);
    ExVectorNormal(&cross);
    
    Camera.AxisX = ExRotateVectorAroundAxis(&Camera.AxisX, value, &cross);
    Camera.AxisY = ExVectorCross(&cross, &Camera.AxisX);
    ExVectorNormal(&Camera.AxisY);
}

/** @brief Fast length of 3D vector (Thx GValiente for solution).
 *  For more info about how it works see: https://math.stackexchange.com/questions/1282435/alpha-max-plus-beta-min-algorithm-for-three-numbers
 *  And also: https://en.wikipedia.org/wiki/Alpha_max_plus_beta_min_algorithm
 *  @param vector Vector to measure
 *  @return Approximation of the vector length
 */
jo_fixed FastVectorLength(const jo_vector_fixed * vector)
{
    // Alpha is 0.9398086351723256
    // Beta is 0.38928148272372454
    // Gama is 0.2987061876143797

    // Get absolute values of the vector components
    jo_fixed x = JO_ABS(vector->x);
    jo_fixed y = JO_ABS(vector->y);
    jo_fixed z = JO_ABS(vector->z);

    // Get min, mid, max
    jo_fixed minYZ = JO_MIN(y, z);
    jo_fixed maxYZ = JO_MAX(y, z);
    jo_fixed min = JO_MIN(x, minYZ);
    jo_fixed max = JO_MAX(x, maxYZ);
    jo_fixed mid = (y < x) ? ((y < z) ? ((z < x) ? z : x) : y) : ((x < z) ? ((z < y) ? z : y) : x);
    
    // Aproximate vector length (alpha * max + beta * mid + gama * min)
    jo_fixed approximation = jo_fixed_mult(61591, max) + jo_fixed_mult(25512, mid) + jo_fixed_mult(19576, min);
    return JO_MAX(max, approximation);
}

/** @brief Calculates collision between moving point and static AABB
 *  @param pointPrev Previous position of the moving point
 *  @param pointCur Current position of the moving point
 *  @param aabbCenter Center point of collider box
 *  @param aabbRadius Radius size of the collider box
 *  @param repulsionVector Resulting repulsion vector from collision
 *  @return True if there is collision 
 */
bool SweptCollisionPointToAABB(
    const jo_pos3D_fixed * pointPrev,
    const jo_pos3D_fixed * pointCur,
    const jo_pos3D_fixed * aabbCenter,
    const jo_fixed aabbRadius,
    jo_vector_fixed * repulsionVector)
{
    jo_fixed entryTime;
    jo_fixed entryX;
    jo_fixed entryY;
    jo_fixed entryZ;
    bool collisionX = false;
    bool collisionY = false;
    bool collisionZ = false;

    jo_vector_fixed movementVector = { { pointCur->x - pointPrev->x, pointCur->y - pointPrev->y, pointCur->z - pointPrev->z } };

    // Default to maximal entry time (1.0f)
    repulsionVector->x = JO_FIXED_1;
    repulsionVector->y = JO_FIXED_1;
    repulsionVector->z = JO_FIXED_1;

    // Box sides
    jo_fixed right = aabbCenter->x + aabbRadius;
    jo_fixed left = aabbCenter->x - aabbRadius;

    jo_fixed forward = aabbCenter->y + aabbRadius;
    jo_fixed backward = aabbCenter->y - aabbRadius;

    jo_fixed top = aabbCenter->z + aabbRadius;
    jo_fixed bottom = aabbCenter->z - aabbRadius;

    // Check for collision on X axis line
    if (movementVector.x != 0)
    {
        if ((movementVector.x > 0 && pointPrev->x <= left && pointCur->x > left) ||
             (movementVector.x < 0 && pointPrev->x >= right && pointCur->x < right))
        {
            // We got collision
            if (movementVector.x > 0)
            {
                entryTime = left - pointPrev->x;
            }
            else
            {
                entryTime = right - pointPrev->x;
            }

            // Calculate entry time (distance from previous position to the box along movement direction)
            entryTime = jo_fixed_div(entryTime, movementVector.x);
            entryY = jo_fixed_mult(movementVector.y, entryTime);
            entryZ = jo_fixed_mult(movementVector.z, entryTime);

            // Check for Y and Z collision
            if (pointPrev->y + entryY <= forward && 
                pointPrev->y + entryY >= backward &&
                pointPrev->z + entryZ <= top && 
                pointPrev->z + entryZ >= bottom)
            {
                repulsionVector->x = entryTime;
                collisionX = true;
            }
        }
    }

    // Check for collision on Y axis line
    if (movementVector.y != 0)
    {
        if ((movementVector.y > 0 && pointPrev->y <= backward && pointCur->y > backward) ||
            (movementVector.y < 0 && pointPrev->y >= forward && pointCur->y < forward))
        {
            // We got collision
            if (movementVector.y > 0)
            {
                entryTime = backward - pointPrev->y;
            }
            else
            {
                entryTime = forward - pointPrev->y;
            }

            // Calculate entry time (distance from previous position to the box along movement direction)
            entryTime = jo_fixed_div(entryTime, movementVector.y);
            entryX = jo_fixed_mult(movementVector.x, entryTime);
            entryZ = jo_fixed_mult(movementVector.z, entryTime);

            // Check for X and Z collision
            if (pointPrev->x + entryX <= right &&
                pointPrev->x + entryX >= left &&
                pointPrev->z + entryZ <= top && 
                pointPrev->z + entryZ >= bottom)
            {
                repulsionVector->y = entryTime;
                collisionY = true;
            }
        }
    }
        
    // Check for collision on Z axis line
    if (movementVector.z != 0)
    {
        if ((movementVector.z > 0 && pointPrev->z <= bottom && pointCur->z > bottom) ||
            (movementVector.z < 0 && pointPrev->z >= top && pointCur->z < top))
        {
            // We got collision
            if (movementVector.z > 0)
            {
                entryTime = bottom - pointPrev->z;
            }
            else
            {
                entryTime = top - pointPrev->z;
            }

            // Calculate entry time (distance from previous position to the box along movement direction)
            entryTime = jo_fixed_div(entryTime, movementVector.z);
            entryX = jo_fixed_mult(movementVector.x, entryTime);
            entryY = jo_fixed_mult(movementVector.y, entryTime);

            // Check for X and Y collision
            if (pointPrev->x + entryX <= right &&
                pointPrev->x + entryX >= left &&
                pointPrev->y + entryY <= forward && 
                pointPrev->y + entryY >= backward)
            {
                repulsionVector->z = entryTime;
                collisionZ = true;
            }
        }
    }

    return collisionX || collisionY || collisionZ;
}

/** @brief Do collision with PhysPoint
 *  @param index PhysPoint index
 *  @return True if collides with box
 */
bool DoCollideWithClothPoint(int index)
{
    bool collision = false;
    int meshPoint = Points[index].PosIndex;

    jo_pos3D_fixed current;
    current.x = MeshPoints[meshPoint][X];
    current.y = MeshPoints[meshPoint][Y];
    current.z = MeshPoints[meshPoint][Z];
    current.z = MAX(current.z, 0);

    jo_vector_fixed result;

    if (SweptCollisionPointToAABB(&Points[index].PrevPos, &current, &BoxPosition, BOX_SIZE << 16, &result))
    {
        current.x = Points[index].PrevPos.x + jo_fixed_mult(current.x - Points[index].PrevPos.x, result.x);
        current.y = Points[index].PrevPos.y + jo_fixed_mult(current.y - Points[index].PrevPos.y, result.y);
        current.z = Points[index].PrevPos.z + jo_fixed_mult(current.z - Points[index].PrevPos.z, result.z);
        collision = true;
    }

    MeshPoints[meshPoint][X] = current.x;
    MeshPoints[meshPoint][Y] = current.y;
    MeshPoints[meshPoint][Z] = current.z;
    return collision;
}

/** @brief Use controller to move camera 
 */
void MoveCamera()
{
    if (jo_is_pad1_key_pressed(JO_KEY_A))
    {
        DoThrust(MOVE_SPEED);
    }
    else if (jo_is_pad1_key_pressed(JO_KEY_B))
    {
        DoThrust(-MOVE_SPEED);
    }

    // Yaw
    if (jo_is_pad1_key_pressed(JO_KEY_LEFT))
    {
        DoYaw(TURN_SPEED);
    }
    else if (jo_is_pad1_key_pressed(JO_KEY_RIGHT))
    {
        DoYaw(-TURN_SPEED);
    }
    
    // Pitch
    if (jo_is_pad1_key_pressed(JO_KEY_UP))
    {
        DoPitch(TURN_SPEED);
    }
    else if (jo_is_pad1_key_pressed(JO_KEY_DOWN))
    {
        DoPitch(-TURN_SPEED);
    }
    
    // Roll
    if (jo_is_pad1_key_pressed(JO_KEY_L))
    {
        DoRoll(TURN_SPEED);
    }
    else if (jo_is_pad1_key_pressed(JO_KEY_R))
    {
        DoRoll(-TURN_SPEED);
    }
}

/** @brief Use controller to move cloth 
 */
void MoveCloth()
{
    int movex = 0;
    int movey = 0;
    int movez = 0;

    if (jo_is_pad1_key_pressed(JO_KEY_LEFT))
    {
        movex--;
    }
    else if (jo_is_pad1_key_pressed(JO_KEY_RIGHT))
    {
        movex++;
    }

    if (jo_is_pad1_key_pressed(JO_KEY_UP))
    {
        movey--;
    }
    else if (jo_is_pad1_key_pressed(JO_KEY_DOWN))
    {
        movey++;
    }

    if (jo_is_pad1_key_pressed(JO_KEY_L))
    {
        movez--;
    }
    else if (jo_is_pad1_key_pressed(JO_KEY_R))
    {
        movez++;
    }

    if (movex != 0 || movey != 0 || movez != 0)
    {
        for(int pointx = 0; pointx < CLOTH_SIZE_W; pointx++)
        {
            for(int pointy = 0; pointy < CLOTH_SIZE_H; pointy++)
            {
                // Generate point
                int coord = pointx + (CLOTH_SIZE_W * pointy);

                if (Points[coord].Locked)
                {
                    jo_pos3D_fixed current;
                    current.x = MeshPoints[Points[coord].PosIndex][X] + (jo_int2fixed(movex) >> 1);
                    current.y = MeshPoints[Points[coord].PosIndex][Y] + (jo_int2fixed(movey) >> 1);
                    current.z = MeshPoints[Points[coord].PosIndex][Z] + (jo_int2fixed(movez) >> 1);
                    current.z = MAX(current.z, 0);

                    jo_vector_fixed result;

                    if (SweptCollisionPointToAABB(&Points[coord].PrevPos, &current, &BoxPosition, BOX_SIZE << 16, &result))
                    {
                        current.x = Points[coord].PrevPos.x + jo_fixed_mult(current.x - Points[coord].PrevPos.x, result.x);
                        current.y = Points[coord].PrevPos.y + jo_fixed_mult(current.y - Points[coord].PrevPos.y, result.y);
                        current.z = Points[coord].PrevPos.z + jo_fixed_mult(current.z - Points[coord].PrevPos.z, result.z);
                    }

                    MeshPoints[Points[coord].PosIndex][X] = current.x;
                    MeshPoints[Points[coord].PosIndex][Y] = current.y;
                    MeshPoints[Points[coord].PosIndex][Z] = current.z;
                    Points[coord].PrevPos.x = MeshPoints[Points[coord].PosIndex][X];
                    Points[coord].PrevPos.y = MeshPoints[Points[coord].PosIndex][Y];
                    Points[coord].PrevPos.z = MeshPoints[Points[coord].PosIndex][Z];
                }
            }
        }
    }
}

/** @brief Simulate cloth
 */
void DemoClothSim()
{
    // Update points
    int count = CLOTH_SIZE_W * CLOTH_SIZE_H;

    for(int point = 0; point < count; point++)
    {
        if (!Points[point].Locked)
        {
            int posIndex = Points[point].PosIndex;

            // Do box collision
            bool collidesWithBox = DoCollideWithClothPoint(point);
            
            // Do gravity
            jo_pos3D_fixed prev = { MeshPoints[posIndex][X], MeshPoints[posIndex][Y], MeshPoints[posIndex][Z] };
            MeshPoints[posIndex][X] += (MeshPoints[posIndex][X] - Points[point].PrevPos.x) + Gravity.x;
            MeshPoints[posIndex][Y] += (MeshPoints[posIndex][Y] - Points[point].PrevPos.y) + Gravity.y;
            MeshPoints[posIndex][Z] += (MeshPoints[posIndex][Z] - Points[point].PrevPos.z) + Gravity.z;

            MeshPoints[posIndex][Z] = MAX(MeshPoints[posIndex][Z], 0);

            // Make cloth drag on stuff by making the movement force from last frame half
            if ((collidesWithBox && CubeIsSticky) || MeshPoints[posIndex][Z] == 0)
            {
                MeshPoints[posIndex][X] = (MeshPoints[posIndex][X] + prev.x) >> 1;
                MeshPoints[posIndex][Y] = (MeshPoints[posIndex][Y] + prev.y) >> 1;
                MeshPoints[posIndex][Z] = (MeshPoints[posIndex][Z] + prev.z) >> 1;
            }
            
            Points[point].PrevPos.x = prev.x;
            Points[point].PrevPos.y = prev.y;
            Points[point].PrevPos.z = prev.z;
        }
    }

    count = (CLOTH_SIZE_W * (CLOTH_SIZE_H - 1)) + ((CLOTH_SIZE_W - 1) * CLOTH_SIZE_H);

    for (int segment = 0; segment < count; segment++)
    {
        // Skip segments where both points are locked
        if ((!Segments[segment].First->Locked || !Segments[segment].Second->Locked))
        {
            int firstIndex = Segments[segment].First->PosIndex;
            int secondIndex = Segments[segment].Second->PosIndex;

            jo_vector_fixed dir;
            dir.x = MeshPoints[firstIndex][X] - MeshPoints[secondIndex][X];
            dir.y = MeshPoints[firstIndex][Y] - MeshPoints[secondIndex][Y];
            dir.z = MeshPoints[firstIndex][Z] - MeshPoints[secondIndex][Z];
            jo_fixed length = FastVectorLength(&dir);

            // Segment is overstretched
            if (length > Segments[segment].Length)
            {
                // Normalize vector
                dir.x = jo_fixed_div(dir.x, length);
                dir.y = jo_fixed_div(dir.y, length);
                dir.z = jo_fixed_div(dir.z, length);

                // Segment center point
                jo_pos3D_fixed center;
                center.x = (MeshPoints[firstIndex][X] + MeshPoints[secondIndex][X]) >> 1;
                center.y = (MeshPoints[firstIndex][Y] + MeshPoints[secondIndex][Y]) >> 1;
                center.z = (MeshPoints[firstIndex][Z] + MeshPoints[secondIndex][Z]) >> 1;

                jo_vector_fixed segmentDir;
                segmentDir.x = jo_fixed_mult(dir.x, Segments[segment].Length) >> 1;
                segmentDir.y = jo_fixed_mult(dir.y, Segments[segment].Length) >> 1;
                segmentDir.z = jo_fixed_mult(dir.z, Segments[segment].Length) >> 1;

                // Don't move point if locked
                if (!Segments[segment].First->Locked)
                {
                    MeshPoints[firstIndex][X] = center.x + segmentDir.x;
                    MeshPoints[firstIndex][Y] = center.y + segmentDir.y;
                    MeshPoints[firstIndex][Z] = center.z + segmentDir.z;
                    MeshPoints[firstIndex][Z] = MAX(MeshPoints[firstIndex][Z], 0);
                }

                // Don't move point if locked
                if (!Segments[segment].Second->Locked)
                {
                    MeshPoints[secondIndex][X] = center.x - segmentDir.x;
                    MeshPoints[secondIndex][Y] = center.y - segmentDir.y;
                    MeshPoints[secondIndex][Z] = center.z - segmentDir.z;
                    MeshPoints[secondIndex][Z] = MAX(MeshPoints[secondIndex][Z], 0);
                }
            }
        }
    }
}

/** @brief Main demo loop
 */
void DemoLogic()
{
    // Handle gamepad
    if (jo_is_pad1_available())
    {
        // Change control mode
        if (jo_is_pad1_key_down(JO_KEY_C))
        {
            ControlMode += 1;

            if (ControlMode > 1)
            {
                ControlMode = 0;
            }
        }

        // Cycle cloth texture
        if (jo_is_pad1_key_down(JO_KEY_X))
        {
            ClothTexture += 1;

            if (ClothTexture > MAX_CLOTH_TEXTURES - 1)
            {
                ClothTexture = 0;
            }

            jo_3d_set_mesh_texture(&Cloth, ClothTexture);
        }

        // Cycle cloth texture
        if (jo_is_pad1_key_down(JO_KEY_Y))
        {
            CubeIsSticky = !CubeIsSticky;
        }

        // Process input
        switch (ControlMode)
        {
            case 0:
                MoveCloth();
                break;

            case 1:
                MoveCamera();
                break;

            default:
                break;
        }
    }

    // Do simulation
    DemoClothSim();
}

/** @brief Redering loop
 */
void DemoDraw()
{
    // Simple UI
    jo_printf_with_color(1,1, JO_COLOR_INDEX_White, "Mode: %s          ", ControlMode == 0 ? "Moving cloth" : "Camera");
    jo_printf(1,3, "Controls:");

    // Cycle cloth textures
    jo_printf(1,4, "Cycle cloth texture: X (current: %d)", ClothTexture);
    jo_printf(1,5, "Cloth drags on cube: Y (current: %s)", CubeIsSticky ? "True " : "False");

    if (ControlMode == 0)
    {
        jo_printf(1,6, "Switch to camera mode: C    ");
        jo_printf(1,7, "Move on plane: D-Pad        ");
        jo_printf(1,8, "Move up/down: R/L           ");
        jo_printf(1,9, "                            ");
        jo_printf(1,10, "                            ");
        jo_printf(1,11, "                            ");

    }
    else
    {
        jo_printf(1,6, "Switch to cloth mode: C     ");
        jo_printf(1,7, "Yaw: D-Pad left/right       ");
        jo_printf(1,8, "Pitch: D-Pad up/down        ");
        jo_printf(1,9, "Roll: R/L");
        jo_printf(1,10, "Move forward: A");
        jo_printf(1,11, "Move backward: B");
    }

    // Credits
    jo_printf_with_color(1,28, JO_COLOR_INDEX_Red, "Demo by SuperReye (www.reye.me)");

    // Set 3D matrix
    MATRIX matrix;
    ExGetMatrixToCoordinateSystem(&Camera, matrix);

    jo_3d_push_matrix();
	{
        slLoadMatrix(matrix);

        // Draw cloth mesh
        jo_3d_mesh_draw(&Cloth);

        // Draw box
        jo_3d_push_matrix();
        {
            jo_3d_translate_matrix_fixed(BoxPosition.x, BoxPosition.y, BoxPosition.z);
            jo_3d_draw_array(BoxQuads, 6);
        }
        jo_3d_pop_matrix();

        // Draw ground
        jo_background_3d_plane_a_draw(true);
	}
	jo_3d_pop_matrix();

    jo_3d_push_matrix();
    {
        jo_3d_translate_matrix_z(-32);
        
        // Draw sky
        jo_background_3d_plane_b_draw(false);
    }
    jo_3d_pop_matrix();
}

/** @brief Palette loader for RGB0
 */
static jo_palette *Rgb0PaletteHandling(void)
{
    // We re-create a new palette for each image. It's not optimal and WILL break with different images that don't have same palette but OK for a demo.
    jo_create_palette(&Rgb0Palette);
    return (&Rgb0Palette);
}

/** @brief Load assets
 */
void LoadAssets()
{
    jo_core_tv_off();

    // set palette loading
    jo_set_tga_palette_handling(Rgb0PaletteHandling);

    // Enable RGB0
    jo_enable_background_3d_plane(JO_COLOR_Blue);

    // Load water layer texture
    jo_img_8bits img;
    img.data = JO_NULL;
    jo_tga_8bits_loader(&img, JO_ROOT_DIR, "GROUND.TGA", 0);
    jo_background_3d_plane_a_img(&img, Rgb0Palette.id, true, true);
    jo_free_img(&img);

    // Load sky texture
    img.data = JO_NULL;
    jo_tga_8bits_loader(&img, JO_ROOT_DIR, "SKY.TGA", 0);
    jo_background_3d_plane_b_img(&img, Rgb0Palette.id, true, false);
    jo_free_img(&img);

    // Load cloth textures (modify MAX_CLOTH_TEXTURES if oyu want to add more)
    jo_sprite_add_tga(JO_ROOT_DIR, "CLOTH.TGA", JO_COLOR_Transparent);
    jo_sprite_add_tga(JO_ROOT_DIR, "CLOTH1.TGA", JO_COLOR_Transparent);
    jo_sprite_add_tga(JO_ROOT_DIR, "CLOTH2.TGA", JO_COLOR_Transparent);
    jo_sprite_add_tga(JO_ROOT_DIR, "CLOTH3.TGA", JO_COLOR_Transparent);

    // Load box texture
    BoxTextureIndex = jo_sprite_add_tga(JO_ROOT_DIR, "BOX.TGA", JO_COLOR_Transparent);

    jo_core_tv_on();
}

/** @brief Initialize demo data
 */
void DemoInitialize()
{
    Gravity.x = 0;
    Gravity.y = 0;
    Gravity.z = -1700;

    // Generate box
    jo_3d_create_cube(BoxQuads, BoxVertices);

    for (int quad = 0; quad < 6; ++quad)
    {
        jo_3d_set_texture(&BoxQuads[quad], BoxTextureIndex);
        BoxQuads[quad].data.attbl[0].sort = (SORT_MAX) | (((sprNoflip) >> 16) & 0x1c) | (No_Option);
    }

    // Generate cloth mesh
    int segment = 0;

    for(int pointx = 0; pointx < CLOTH_SIZE_W; pointx++)
    {
        bool canLock = pointx % 4 == 0;

        for(int pointy = 0; pointy < CLOTH_SIZE_H; pointy++)
        {
            // Generate point
            int x = pointx * CLOTH_SPACING;
            int z = (pointy * CLOTH_SPACING);
            int coord = pointx + (CLOTH_SIZE_W * pointy);

            MeshPoints[coord][X] = x;
            MeshPoints[coord][Y] = 0;
            MeshPoints[coord][Z] = z;
            Points[coord].PosIndex = coord;
            Points[coord].PrevPos.x = x + (jo_random(1000) - 500);
            Points[coord].PrevPos.y = (jo_random(1000) - 500);
            Points[coord].PrevPos.z = z;

            Points[coord].Locked = pointy == CLOTH_SIZE_H - 1 && canLock;

            // Create segment
            if (pointx < CLOTH_SIZE_W - 1)
            {
                Segments[segment].First = &(Points[coord]);
                Segments[segment].Second = &(Points[coord + 1]);
                Segments[segment].Length = CLOTH_SPACING;
                segment++;
            }

            if (pointy < CLOTH_SIZE_H - 1)
            {
                Segments[segment].First = &(Points[coord]);
                Segments[segment].Second = &(Points[pointx + (CLOTH_SIZE_W * (pointy + 1))]);
                Segments[segment].Length = CLOTH_SPACING;
                segment++;
            }
        }
    }

    // Initialize mesh
    Cloth.data.nbPoint = CLOTH_SIZE_H * CLOTH_SIZE_W;
    Cloth.data.pntbl = MeshPoints;
    Cloth.data.nbPolygon = (CLOTH_SIZE_H - 1) * (CLOTH_SIZE_W - 1);
    Cloth.data.attbl = jo_malloc(sizeof(ATTR) * Cloth.data.nbPolygon);
    Cloth.data.pltbl = jo_malloc(sizeof(POLYGON) * Cloth.data.nbPolygon);

    for (Uint32 attribute = 0; attribute < Cloth.data.nbPolygon; attribute++)
    {
        ATTR attr = ATTRIBUTE(
            Dual_Plane,
            SORT_CEN,
            0,
            JO_COLOR_RGB(255, 255, 255),
            CL32KRGB | No_Gouraud,
            CL32KRGB | MESHoff,
            sprNoflip,
            No_Option);

        Cloth.data.attbl[attribute] = attr;
    }
    
    for (int polyx = 0; polyx < CLOTH_SIZE_W - 1; polyx++)
    {
        for (int polyy = 0; polyy < CLOTH_SIZE_H - 1; polyy++)
        {
            int coord = polyx + ((CLOTH_SIZE_W - 1) * polyy);
            int coord1 = polyx + (CLOTH_SIZE_W * polyy);
            int coord2 = polyx + (CLOTH_SIZE_W * polyy) + 1;
            int coord3 = polyx + (CLOTH_SIZE_W * (polyy + 1)) + 1;
            int coord4 = polyx + (CLOTH_SIZE_W * (polyy + 1));

            Cloth.data.pltbl[coord].Vertices[0] = coord1;
            Cloth.data.pltbl[coord].Vertices[1] = coord2;
            Cloth.data.pltbl[coord].Vertices[2] = coord3;
            Cloth.data.pltbl[coord].Vertices[3] = coord4;

            Cloth.data.pltbl[coord].norm[X] = 0;
            Cloth.data.pltbl[coord].norm[Y] = 0;
            Cloth.data.pltbl[coord].norm[Z] = JO_FIXED_1;
        }
    }
}

/** @brief Application entry point
 */
void jo_main(void)
{
    // Prepare scene
    jo_core_init(JO_COLOR_Black);
    *(volatile unsigned char *)0x060FFCD8 = 0x1F;

    jo_core_set_restart_game_callback(jo_goto_boot_menu);

    LoadAssets();
    DemoInitialize();

    // Start game
    jo_core_add_callback(DemoDraw);
    jo_core_add_callback(DemoLogic);

    jo_core_run();
}
