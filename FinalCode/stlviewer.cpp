#include <vector>
#include <iostream>
#include "fssimplewindow.h"
#include "ysclass.h"

void Add4D(std::vector <GLfloat> &data,GLfloat r,GLfloat g,GLfloat b,GLfloat a)
{
	data.push_back(r);
	data.push_back(g);
	data.push_back(b);
	data.push_back(a);
}
void Add3D(std::vector <GLfloat> &data,GLfloat x,GLfloat y,GLfloat z)
{
	data.push_back(x);
	data.push_back(y);
	data.push_back(z);
}

void GetBoundingBox(YsVec3 minmax[2],const std::vector <GLfloat> &vtx)
{
	if(3<=vtx.size())
	{
		minmax[0].Set(vtx[0],vtx[1],vtx[2]);
		minmax[1].Set(vtx[0],vtx[1],vtx[2]);
		for(int i=3; i+3<=vtx.size(); i+=3)
		{
			auto x=vtx[i];
			auto y=vtx[i+1];
			auto z=vtx[i+2];
			minmax[0].SetX(std::min<double>(minmax[0].x(),x));
			minmax[0].SetY(std::min<double>(minmax[0].y(),y));
			minmax[0].SetZ(std::min<double>(minmax[0].z(),z));
			minmax[1].SetX(std::max<double>(minmax[1].x(),x));
			minmax[1].SetY(std::max<double>(minmax[1].y(),y));
			minmax[1].SetZ(std::max<double>(minmax[1].z(),z));
		}
	}
	else
	{
		minmax[0].Set(0.0,0.0,0.0);
		minmax[1].Set(0.0,0.0,0.0);
	}
}

// int main(int argc,char *argv[])
// {
// 	FsOpenWindow(0,0,800,600,1);
// 	std::vector <GLfloat> vtx,nom,col;

// 	if(2<=argc)
// 	{
// 		if(true!=LoadBinSTL(vtx,nom,argv[1]))
// 		{
// 			printf("Read error!\n");
// 			return 1;
// 		}
// 	}
// 	else
// 	{
// 		printf("File name not given.\n");
// 		return 0;
// 	}

// 	//checking to see if data is loading
// 	// Print the number of vertices and normals loaded
// 	printf("Number of vertices: %d\n", vtx.size() / 3); // Assuming each vertex has 3 components (x, y, z)
// 	printf("Number of normals: %d\n", nom.size() / 3); // Assuming each normal has 3 components (x, y, z)
// 	// Print the first few vertices and normals
// 	printf("First few vertices:\n");
// 	for (int i = 0; i < std::min(5, (int)vtx.size()); i += 3) {
// 		printf("(%f, %f, %f)\n", vtx[i], vtx[i + 1], vtx[i + 2]);
// 	}
// 	printf("First few normals:\n");
// 	for (int i = 0; i < std::min(5, (int)nom.size()); i += 3) {
// 		printf("(%f, %f, %f)\n", nom[i], nom[i + 1], nom[i + 2]);
// 	}
// 	//yes data is being loaded

// 	for(int i=0; i<vtx.size()/3; ++i)
// 	{
// 		col.push_back(0);
// 		col.push_back(1);
// 		col.push_back(0);
// 		col.push_back(1);
// 	}

// 	YsVec3 minmax[2];
// 	GetBoundingBox(minmax,vtx);

// 	YsMatrix4x4 Rc;
// 	YsVec3 target=(minmax[0]+minmax[1])/2.0;
// 	double dist=(minmax[1]-minmax[0]).GetLength();

// 	for(;;)
// 	{
// 		FsPollDevice();
// 		auto key=FsInkey();
// 		if(FSKEY_ESC==key)
// 		{
// 			break;
// 		}

// 		if(0!=FsGetKeyState(FSKEY_LEFT))
// 		{
// 			Rc.RotateXZ(0.01);
// 		}
// 		if(0!=FsGetKeyState(FSKEY_RIGHT))
// 		{
// 			Rc.RotateXZ(-0.01);
// 		}
// 		if(0!=FsGetKeyState(FSKEY_UP))
// 		{
// 			Rc.RotateZY(0.01);
// 		}
// 		if(0!=FsGetKeyState(FSKEY_DOWN))
// 		{
// 			Rc.RotateZY(-0.01);
// 		}

// 		int lb,mb,rb,mx,my;
// 		auto evt=FsGetMouseEvent(lb,mb,rb,mx,my);
// 		if(FSMOUSEEVENT_LBUTTONDOWN==evt)
// 		{
// 		}

// 		int wid,hei;
// 		FsGetWindowSize(wid,hei);
// 		double aspect=(double)wid/(double)hei;

// 		glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

// 		glEnable(GL_DEPTH_TEST);

// 		glMatrixMode(GL_PROJECTION);
// 		glLoadIdentity();
// 		gluPerspective(45.0,aspect,0.1,100.0);

// 		glMatrixMode(GL_MODELVIEW);
// 		glLoadIdentity();

// 	    GLfloat lightDir[]={0,1/sqrt(2.0f),1/sqrt(2.0f),0};
// 	    glLightfv(GL_LIGHT0,GL_POSITION,lightDir);
// 	    glEnable(GL_COLOR_MATERIAL);
// 	    glEnable(GL_LIGHTING);
// 	    glEnable(GL_LIGHT0);

// 		////checking lighting and materials
// 		//// Verify lighting configuration
// 		//printf("Light Position: (%f, %f, %f)\n", lightDir[0], lightDir[1], lightDir[2]);
// 		//// Check for OpenGL errors
// 		//GLenum error = glGetError();
// 		//if (error != GL_NO_ERROR) {
// 		//	printf("OpenGL Error: %s\n", gluErrorString(error));
// 		//}
// 		//I don't think there is an error here

// 		auto Rinv=Rc;
// 		Rinv.Invert();

// 		YsMatrix4x4 modelView;
// 		GLdouble modelViewD[16];
// 		modelView.Translate(YsVec3(0,0,-dist));
// 		modelView*=Rinv;
// 		modelView.Translate(-target);

// 		modelView.GetOpenGlCompatibleMatrix(modelViewD);
// 		glMultMatrixd(modelViewD);

// 		if(0<vtx.size())
// 		{
// 			glEnableClientState(GL_VERTEX_ARRAY);
// 			glEnableClientState(GL_NORMAL_ARRAY);
// 			glEnableClientState(GL_COLOR_ARRAY);
// 			glColorPointer(4,GL_FLOAT,0,col.data());
// 			glVertexPointer(3,GL_FLOAT,0,vtx.data());
// 			glNormalPointer(GL_FLOAT,0,nom.data());
// 			glDrawArrays(GL_TRIANGLES,0,vtx.size()/3);
// 			glDisableClientState(GL_VERTEX_ARRAY);
// 			glDisableClientState(GL_NORMAL_ARRAY);
// 			glDisableClientState(GL_COLOR_ARRAY);
// 		}

// 		FsSwapBuffers();

// 		FsSleep(10);
// 	}

// 	printf("*\n");

// 	return 0;
// }
