#include <jo/jo.h>

#define CLOTH_SIZE_W (13)
#define CLOTH_SIZE_H (5)
#define CLOTH_SPACING JO_FIXED_4

#define SIM_ITERATIONS (5)

/** @brief Phys point
 */
typedef struct {
    int PosIndex;
    jo_pos3D_fixed PrevPos;
    int Locked;
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

/** @brief Main demo loop
 */
void DemoLogic()
{
    // Update points
    int count = CLOTH_SIZE_W * CLOTH_SIZE_H;

    for(int point = 0; point < count; point++)
    {
        if (Points[point].Locked == 0)
        {
            int posIndex = Points[point].PosIndex;
            
            jo_pos3D_fixed prev = { MeshPoints[posIndex][X], MeshPoints[posIndex][Y], MeshPoints[posIndex][Z] };
            MeshPoints[posIndex][X] += (MeshPoints[posIndex][X] - Points[point].PrevPos.x) + Gravity.x;
            MeshPoints[posIndex][Y] += (MeshPoints[posIndex][Y] - Points[point].PrevPos.y) + Gravity.y;
            MeshPoints[posIndex][Z] += (MeshPoints[posIndex][Z] - Points[point].PrevPos.z) + Gravity.z;

            Points[point].PrevPos.x = prev.x;
            Points[point].PrevPos.y = prev.y;
            Points[point].PrevPos.z = prev.z;
        }
    }

    count = (CLOTH_SIZE_W * (CLOTH_SIZE_H - 1)) + ((CLOTH_SIZE_W - 1) * CLOTH_SIZE_H);

    for (int iteration = 0; iteration < SIM_ITERATIONS; iteration++)
    {
        for (int segment = 0; segment < count; segment++)
        {
            jo_pos3D_fixed center;
            int firstIndex = Segments[segment].First->PosIndex;
            int secondIndex = Segments[segment].Second->PosIndex;

            center.x = jo_fixed_div(MeshPoints[firstIndex][X] + MeshPoints[secondIndex][X], JO_FIXED_2);
            center.y = jo_fixed_div(MeshPoints[firstIndex][Y] + MeshPoints[secondIndex][Y], JO_FIXED_2);
            center.z = jo_fixed_div(MeshPoints[firstIndex][Z] + MeshPoints[secondIndex][Z], JO_FIXED_2);

            jo_vector_fixed dir;
            dir.x = MeshPoints[firstIndex][X] - MeshPoints[secondIndex][X];
            dir.y = MeshPoints[firstIndex][Y] - MeshPoints[secondIndex][Y];
            dir.z = MeshPoints[firstIndex][Z] - MeshPoints[secondIndex][Z];
            jo_fixed length = jo_fixed_sqrt(jo_fixed_mult(dir.x, dir.x) + jo_fixed_mult(dir.y, dir.y) + jo_fixed_mult(dir.z, dir.z));

            if (length > Segments[segment].Length)
            {
                dir.x = jo_fixed_div(dir.x, length);
                dir.y = jo_fixed_div(dir.y, length);
                dir.z = jo_fixed_div(dir.z, length);

                jo_vector_fixed segmentDir;
                segmentDir.x = jo_fixed_div(jo_fixed_mult(dir.x, Segments[segment].Length), JO_FIXED_2);
                segmentDir.y = jo_fixed_div(jo_fixed_mult(dir.y, Segments[segment].Length), JO_FIXED_2);
                segmentDir.z = jo_fixed_div(jo_fixed_mult(dir.z, Segments[segment].Length), JO_FIXED_2);

                // Don't move point if locked
                if (!Segments[segment].First->Locked)
                {
                    MeshPoints[firstIndex][X] = center.x + segmentDir.x;
                    MeshPoints[firstIndex][Y] = center.y + segmentDir.y;
                    MeshPoints[firstIndex][Z] = center.z + segmentDir.z;
                }

                // Don't move point if locked
                if (!Segments[segment].Second->Locked)
                {
                    MeshPoints[secondIndex][X] = center.x - segmentDir.x;
                    MeshPoints[secondIndex][Y] = center.y - segmentDir.y;
                    MeshPoints[secondIndex][Z] = center.z - segmentDir.z;
                }
            }
        }
    }
    
    int movex = 0;
    int movey = 0;

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

    if (movex != 0 || movey != 0)
    {
        for(int pointx = 0; pointx < CLOTH_SIZE_W; pointx++)
        {
            for(int pointy = 0; pointy < CLOTH_SIZE_H; pointy++)
            {
                // Generate point
                int coord = pointx + (CLOTH_SIZE_W * pointy);

                if (Points[coord].Locked == 1)
                {
                    MeshPoints[Points[coord].PosIndex][X] += jo_int2fixed(movex);
                    MeshPoints[Points[coord].PosIndex][Y] += jo_int2fixed(movey);
                    Points[coord].PrevPos.x = MeshPoints[Points[coord].PosIndex][X];
                    Points[coord].PrevPos.y = MeshPoints[Points[coord].PosIndex][Y];
                }
            }
        }
    }
}

/** @brief Redering loop
 */
void DemoDraw()
{
    // Entities in world
    jo_3d_push_matrix();
	{
        jo_3d_translate_matrix_z(48);
        jo_3d_mesh_draw(&Cloth);
	}
	jo_3d_pop_matrix();
}

/** @brief Initialize demo data
 */
void DemoInitialize()
{
    Gravity.x = 0;
    Gravity.y = 1700;
    Gravity.z = 0;

    jo_sprite_add_tga(JO_ROOT_DIR, "CLOTH.TGA", JO_COLOR_Transparent);
    int segment = 0;

    for(int pointx = 0; pointx < CLOTH_SIZE_W; pointx++)
    {
        for(int pointy = 0; pointy < CLOTH_SIZE_H; pointy++)
        {
            // Generate point
            int x = pointx * CLOTH_SPACING;
            int y = pointy * CLOTH_SPACING;
            int coord = pointx + (CLOTH_SIZE_W * pointy);

            MeshPoints[coord][X] = x;
            MeshPoints[coord][Y] = y;
            MeshPoints[coord][Z] = 0;
            Points[coord].PosIndex = coord;
            Points[coord].PrevPos.x = x;
            Points[coord].PrevPos.y = y;
            Points[coord].PrevPos.z = 0;

            Points[coord].Locked = 0;

            // Create segment
            if (pointx < CLOTH_SIZE_W - 1)
            {
                Segments[segment].First = &(Points[coord]);
                Segments[segment].Second = &(Points[coord + 1]);
                Segments[segment].Length = CLOTH_SPACING + JO_FIXED_1;
                segment++;
            }

            if (pointy < CLOTH_SIZE_H - 1)
            {
                Segments[segment].First = &(Points[coord]);
                Segments[segment].Second = &(Points[pointx + (CLOTH_SIZE_W * (pointy + 1))]);
                Segments[segment].Length = CLOTH_SPACING + JO_FIXED_1;
                segment++;
            }
        }

        if (pointx % 4 == 0)
        {
            Points[pointx].Locked = 1;
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

    DemoInitialize();

    // Start game
    jo_core_add_callback(DemoDraw);
    jo_core_add_callback(DemoLogic);

    jo_core_run();
}
