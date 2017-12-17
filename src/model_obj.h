// Mars lander simulator
// Version 1.8
// Model_obj class header
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

#ifndef __MODEL_OBJ_INCLUDED__
#define __MODEL_OBJ_INCLUDED__

#include "global_1.h"

using namespace std;

class Model_obj
{
  public:
    // Normals : stores {normal_of_face_1.x, normal_of_face_1.y, normal_of_face_1.z, normal_of_face_2.x, normal_of_face_2.y, ...}
    // Face_Vertices : stores {Face_1_Vertex_1.x, Face_1_Vertex_1.y, Face_1_Vertex_1.z, Face_1_Vertex_2.x, Face_1_Vertex_2.y, Face_1_Vertex_2.z, Face_1_Vertex_3.x ,...}
    // UVs : store {Face_1_Vertex_1.u, Face_1_Vertex_1.v, Face_1_Vertex_2.u, Face_1_Vertex_2.v, Face_1_Vertex_3.u, Face_1_Vertex_3.v, Face_2_Vertex_1.u, ...}
    // vertexBuffer : stores {v1.x, v1.y, v1.z, v2.x, v2.y, v2.z, v3.x, ...}
    // texelBuffer: stores {vt1.u, vt1.v, vt2.u, vt2.v, vt3.u, ...}
    // TotalConnectedPoints : total number of elements of vertexBuffer
    // TotalConnectedTexels : total number of elements of texelBuffer
    // TotalConnectedTriangles : total number of elements of Face_Vertices
    float* Normals;
    float* Face_Vertices;
    float* UVs;
    float* vertexBuffer;
    float* texelBuffer;
    long TotalConnectedPoints;
    long TotalConnectedTexels;
    long TotalConnectedTriangles;
    
    Model_obj();
    vector3d calculateNormal(vector3d coord1, vector3d coord2, vector3d coord3);
    bool Load(string filename, float zoom_number);
    void Draw();
    void Release();
};

#endif
