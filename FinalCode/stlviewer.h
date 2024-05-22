#ifndef STLVIEWER_H
#define STLVIEWER_H


#include <vector>
#include <ysclass.h>

void Add4D(std::vector<GLfloat> &data, GLfloat r, GLfloat g, GLfloat b, GLfloat a);
void Add3D(std::vector<GLfloat> &data, GLfloat x, GLfloat y, GLfloat z);
void GetBoundingBox(YsVec3 minmax[2], const std::vector<GLfloat> &vtx);

#endif // STLVIEWER_H
