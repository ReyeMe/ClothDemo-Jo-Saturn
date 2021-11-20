#ifndef __MATHEX__
#define __MATHEX__

/** @brief Coordinate system definition using 2 vectors and a point
 */
typedef struct 
{
    /* Origin point of the coordinate system */
    jo_pos3D_fixed Origin;

    /* Up direction vector */
    jo_vector_fixed AxisX;

    /* Forward direction vector */
    jo_vector_fixed AxisY;
} ExCoordinateSystem;

/** @brief Translate point by vector
 *  @param toTranslate Point to translate
 *  @param delta Translation vector
 */
void ExTranslatePoint(jo_pos3D_fixed * toTranslate, const jo_vector_fixed * delta);

/** @brief Normalize fixed vector
 *  @param a Vector to normalize
 */
void ExVectorNormal(jo_vector_fixed * a);

/** @brief Compute cross product of two vectors
 *  @param a First vector
 *  @param b Second vector
 *  @return Cross product of two vectors
 */
jo_vector_fixed ExVectorCross(const jo_vector_fixed * a, const jo_vector_fixed * b);

/** @brief Compute dot product of two vectors
 *  @param a First vector
 *  @param b Second vector
 *  @return Cross product of two vectors
 */
jo_fixed ExDotProduct(const jo_fixed * a, const jo_fixed * b);

/** @brief Transform point by matrix
 *  @param matrix matrix to transform by
 *  @param point Point to transform
 *  @return result Transformed point
 */
jo_pos3D_fixed ExTransformPoint(const FIXED (*matrix)[XYZ], const jo_pos3D_fixed * point);

/** @brief Get transformation matrix from coordinate system
 *  @param coordSys Coordinate system
 *  @param result Resulting matrix
 */
void ExGetMatrixFromCoordinateSystem(const ExCoordinateSystem * coordSys, FIXED (*result)[3]);

/** @brief Get transformation matrix to coordinate system
 *  @param coordSys Coordinate system
 *  @param result Resulting matrix
 */
void ExGetMatrixToCoordinateSystem(const ExCoordinateSystem * coordSys, FIXED (*result)[XYZ]);

/** @brief Project point onto an axis
 *  @param toProject Point to project
 *  @param origin Axis origin point
 *  @param direction Axis direction
 *  @return Projected point
 */
jo_pos3D_fixed ExProjectPointToAxis(const jo_pos3D_fixed * toProject, const jo_pos3D_fixed * origin, const jo_vector_fixed * direction);

/** @brief Rotate vector around axis
 *  @param toRotate Point to rotate
 *  @param rad Angle to rotate by in radians
 *  @param direction Axis direction
 *  @return Rotated vector
 */
jo_vector_fixed ExRotateVectorAroundAxis(const jo_vector_fixed * toRotate, const jo_fixed rad, const jo_vector_fixed * direction);

/** @brief Rotate point around axis
 *  @param toRotate Point to rotate
 *  @param rad Angle to rotate by in radians
 *  @param origin Axis origin point
 *  @param direction Axis direction
 *  @return Rotated point
 */
jo_pos3D_fixed ExRotatePointAroundAxis(const jo_pos3D_fixed * toRotate, const jo_fixed rad, const jo_pos3D_fixed * origin, const jo_vector_fixed * direction);

#endif
