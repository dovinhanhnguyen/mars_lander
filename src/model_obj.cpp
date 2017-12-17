// Mars lander simulator
// Version 1.8
// Model_obj class implementation
// Anh Nguyen, August 2016

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// dvan2@cam.ac.uk.

// Adapt from the obj loader on http://openglsamples.sourceforge.net/
// to take in UV data.

#include "model_obj.h"

// Model_obj class's member functions

// constructor
Model_obj::Model_obj()
{
  TotalConnectedPoints = 0;
  TotalConnectedTriangles = 0;
  TotalConnectedTexels = 0;
}

// calculate normal of a plane given three points on that plane
vector3d Model_obj::calculateNormal(vector3d coord1, vector3d coord2, vector3d coord3)
{
	return (((coord1-coord2)^(coord1-coord3)).norm());
}

// load obj file and store its data
bool Model_obj::Load(string filename, float zoom_number)
{
  int POINTS_PER_VERTEX = 3;
  int POINTS_PER_TEXEL = 2;
  int TOTAL_FLOATS_IN_TRIANGLE = 9;
  
	string line;
	ifstream objFile(filename.c_str());
	if (objFile.good())
	{
		objFile.seekg(0, ios::end); // go to end of the file,
		long fileSize = objFile.tellg(); // get file size
		objFile.seekg(0, ios::beg); // go back to the beginning of file
    
		vertexBuffer = (float*) malloc (fileSize); // allocate memory for the vertice buffer
    texelBuffer = (float*) malloc (fileSize); // allocate memory for the texel buffer
    
		Face_Vertices = (float*) malloc(fileSize*sizeof(float)); // allocate memory for the triangles
		Normals  = (float*) malloc(fileSize*sizeof(float)); // allocate memory for the normals
    UVs = (float*) malloc(fileSize*sizeof(float)); // allocate memory for the texels
    
		int triangle_index = 0;
		int normal_index = 0;
    int texel_index = 0;
    
		while (!objFile.eof())
		{		
      getline (objFile,line);
      
			if (line.c_str()[0] == 'v' && line.c_str()[1] == ' ') // first two charactes are 'v ' : a vertex coordinate is stored on this line
			{
				line[0] = ' '; // set first character to 0, this will allow us to use sscanf and prevent further search from stopping at this line
 
				sscanf(line.c_str(),"%f %f %f", &vertexBuffer[TotalConnectedPoints], &vertexBuffer[TotalConnectedPoints+1], &vertexBuffer[TotalConnectedPoints+2]); // read floats from the line: v vertex.x vertex.y vertex.z
        
        // scale to correct length
        vertexBuffer[TotalConnectedPoints] *= (MARS_RADIUS/zoom_number);
				vertexBuffer[TotalConnectedPoints + 1] *= (MARS_RADIUS/zoom_number);
				vertexBuffer[TotalConnectedPoints + 2] *= (MARS_RADIUS/zoom_number);
 
				TotalConnectedPoints += POINTS_PER_VERTEX; // add 3 to the total number of elements of vertexBuffer
			}
      
      if (line.c_str()[1] == 't') // first two characters are 'vt' : a texture coordinate is stored on this line
			{
				line[0] = ' '; // set first character to 0, this will allow us to use sscanf and prevent further search from stopping at this line
        line[1] = ' '; // set second character to 0, this will allow us to use sscanf and prevent further search from stopping at this line
 
				sscanf(line.c_str(),"%f %f", &texelBuffer[TotalConnectedTexels], &texelBuffer[TotalConnectedTexels+1]); // read floats from the line: vt UV.u UV.v
 
				TotalConnectedTexels += POINTS_PER_TEXEL; // add 2 to the total number of elements of texelBuffer
			}
      
			if (line.c_str()[0] == 'f') // first character is an 'f': this line stores (vertex_1/UV_1) (vertex_2/UV_2) (vertex_3/UV_3) that define a triangle
			{
        line[0] = ' '; // set first character to 0, this will allow us to use sscanf and prevent further search from stopping at this line
        
				int vertexNumber[4] = {0, 0, 0};
        int texelNumber[4] = {0, 0, 0};
        
        sscanf(line.c_str(),"%d/%d %d/%d %d/%d", &vertexNumber[0], &texelNumber[0], &vertexNumber[1], &texelNumber[1], &vertexNumber[2], &texelNumber[2]);
        // read integers from the line:  f  (vertex_1/UV_1)  (vertex_2/UV_2)  (vertex_3/UV_3)
        
				vertexNumber[0] -= 1; // obj file starts counting from 1
				vertexNumber[1] -= 1; // obj file starts counting from 1
				vertexNumber[2] -= 1; // obj file starts counting from 1
        
        texelNumber[0] -= 1; // obj file starts counting from 1
				texelNumber[1] -= 1; // obj file starts counting from 1
				texelNumber[2] -= 1; // obj file starts counting from 1
        
        // feed vertex coordinate data of the triangle to Face_Vertices 
				int tCounter = 0;
				for (int i = 0; i < POINTS_PER_VERTEX; i++)					
				{
					Face_Vertices[triangle_index+tCounter] = vertexBuffer[3*vertexNumber[i]];
					Face_Vertices[triangle_index+tCounter+1] = vertexBuffer[3*vertexNumber[i]+1];
					Face_Vertices[triangle_index+tCounter+2] = vertexBuffer[3*vertexNumber[i]+2];
					tCounter += POINTS_PER_VERTEX;
				}
        
        // calculate the normal vector of this triangle
				vector3d coord1 = vector3d(Face_Vertices[triangle_index], Face_Vertices[triangle_index+1], Face_Vertices[triangle_index+2]);
				vector3d coord2 = vector3d(Face_Vertices[triangle_index+3], Face_Vertices[triangle_index+4], Face_Vertices[triangle_index+5]);
				vector3d coord3 = vector3d(Face_Vertices[triangle_index+6], Face_Vertices[triangle_index+7], Face_Vertices[triangle_index+8]);
				vector3d norm = calculateNormal(coord1, coord2, coord3);
        
        // feed this normal data to Normals
				tCounter = 0;
				for (int i = 0; i < POINTS_PER_VERTEX; i++)
				{
					Normals[normal_index+tCounter] = norm.x;
					Normals[normal_index+tCounter+1] = norm.y;
					Normals[normal_index+tCounter+2] = norm.z;
					tCounter += POINTS_PER_VERTEX;
				}
        
        // feed UV data of this triangle to UVs
        tCounter = 0;
				for (int i = 0; i < POINTS_PER_VERTEX; i++)
				{
					UVs[texel_index+tCounter] = 1.0-texelBuffer[2*texelNumber[i]]; // somehow texture image is inverted, so need '1.0-' here
					UVs[texel_index+tCounter+1] = 1.0-texelBuffer[2*texelNumber[i]+1]; // somehow texture image is inverted, so need '1.0-' here
					tCounter += POINTS_PER_TEXEL;
				}
        
        // increment indices of arrays
				triangle_index += TOTAL_FLOATS_IN_TRIANGLE;
				normal_index += TOTAL_FLOATS_IN_TRIANGLE;
        texel_index += 6;
				TotalConnectedTriangles += TOTAL_FLOATS_IN_TRIANGLE;
			}
		}
		
    objFile.close(); // close OBJ file
	}
	else 
	{
		cout << "Error while opening file";
    return false;								
	}
	return true;
}

// 'destructor'
void Model_obj::Release()
{
  free(this->Face_Vertices);
  free(this->Normals);
  free(this->UVs);
  free(this->vertexBuffer);
  free(this->texelBuffer);
}

// rendering function
void Model_obj::Draw()
{
  glEnableClientState(GL_VERTEX_ARRAY); // enable vertex array
 	glEnableClientState(GL_NORMAL_ARRAY); // enable normal array
  glEnableClientState(GL_TEXTURE_COORD_ARRAY); // enable UV array
  
	glVertexPointer(3, GL_FLOAT,	0, Face_Vertices); // vertex pointer to Face_Vertices array
  glTexCoordPointer(2, GL_FLOAT, 0, UVs); // uv pointer to UVs arrays
	glNormalPointer(GL_FLOAT, 0, Normals); // normal pointer to Normals array
  
	glDrawArrays(GL_TRIANGLES, 0, TotalConnectedTriangles); // draw the triangles
  
	glDisableClientState(GL_VERTEX_ARRAY); // disable vertex array
	glDisableClientState(GL_NORMAL_ARRAY); // disable normal array
  glDisableClientState(GL_TEXTURE_COORD_ARRAY); // disable UV array
}

