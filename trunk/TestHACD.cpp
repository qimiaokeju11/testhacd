#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <conio.h>

#pragma warning(disable:4996 4100)

#include "wavefront.h"
#include "HACD.h"
#include "PxSimpleTypes.h"
#include "FloatMath.h"

#ifdef WIN32
#define USE_MESH_IMPORT 1
#endif

#if USE_MESH_IMPORT
#include "MeshImport.h"
#include "windows.h"
#endif

namespace HACD
{
	HACD_API *gHACD = NULL;
};

static char * lastSlash(char *path)
{
	char *slash = NULL;
	while ( path )
	{
		path = strchr(path,'\\');
		if ( path )
		{
			slash = path;
			path++;
		}
	}
	return slash;
}

float getFloatArg(int arg,int argc,const char **argv)
{
	float ret = 0;
	if ( arg < argc )
	{
		ret = (float)atof(argv[arg]);
	}
	else
	{
		printf("Error: Missing input argument value at argument location %d.\r\n",arg+1);
	}
	return ret;
}

int getIntArg(int arg,int argc,const char **argv)
{
	int ret = 0;
	if ( arg < argc )
	{
		ret = atoi(argv[arg]);
	}
	else
	{
		printf("Error: Missing input argument value at argument location %d.\r\n",arg+1);
	}
	return ret;
}


void main(int argc,const char ** argv)
{
	if ( argc == 1 )
	{
		printf("Usage: TestHACD <wavefront.obj> (options)\r\n");
		printf("\r\n");
		printf("Options:\r\n");
		printf("-v		: Max Hull Vertices (default 64)\r\n");
		printf("-c		: Concavity (default 100)\r\n");
		printf("-m		: Mimimum number of hulls (default 2)\r\n");
		printf("-merge	: Specifies the merge percentage.  Default is zero.\r\n");
		printf("\r\n");
		printf("Example: TestHACD hornbug.obj -c 500 -m 5\r\n");
		printf("\r\n");
	}
	else
	{
		HACD::HACD_API::Desc desc;
		const char *wavefront = argv[1];
		int scan = 2;
		while ( scan < argc )
		{
			const char *option = argv[scan];
			if ( strcmp(option,"-v") == 0 )
			{
				desc.mMaxHullVertices = getIntArg(scan+1,argc,argv);
				scan+=2;
			}
			else if ( strcmp(option,"-c") == 0 )
			{
				desc.mConcavity = getFloatArg(scan+1,argc,argv);
				scan+=2;
			}
			else if ( strcmp(option,"-m") == 0 )
			{
				desc.mMinHullCount = getIntArg(scan+1,argc,argv);
				scan+=2;
			}
			else if ( strcmp(option,"-merge") == 0 )
			{
				desc.mMergePercentage = getFloatArg(scan+1,argc,argv);
				scan+=2;
			}
		}

		HACD::gHACD = HACD::createHACD_API();
		if  ( HACD::gHACD )
		{
#if USE_MESH_IMPORT
			HMODULE hModule = GetModuleHandleW(NULL);
			char path[MAX_PATH];
			GetModuleFileNameA(hModule, path, MAX_PATH);
			char *slash = lastSlash(path);
			if ( slash )
			{
				*slash = 0;
			}
			printf("Loading MeshImporter DLL's from directory '%s'\r\n", path);
			physx::MeshImport *meshImport = physx::loadMeshImporters(path);
			physx::MeshSystemContainer *msc = NULL;
			STDNAME::vector< physx::PxU32 > indices;
			physx::fm_VertexIndex *vertices = NULL;
			if ( meshImport )
			{
				printf("Opening input mesh file '%s' for read access.\r\n", wavefront );
				FILE *fph = fopen(wavefront,"rb");
				if ( fph )
				{
					fseek(fph,0L,SEEK_END);
					physx::PxU32 len = ftell(fph);
					fseek(fph,0L,SEEK_SET);
					if ( len )
					{
						void *buffer = PX_ALLOC(len);
						fread(buffer,len,1,fph);
						fclose(fph);
						msc = meshImport->createMeshSystemContainer(wavefront,buffer,len,NULL);
						if ( msc )
						{
							physx::MeshSystem *ms = meshImport->getMeshSystem(msc);
							// ok, we new need to convert the input graphics mesh into a single indexed triangle list.
							vertices = physx::fm_createVertexIndex(0.0001f,false);
							for (physx::PxU32 i=0; i<ms->mMeshCount; i++)
							{
								physx::Mesh *m = ms->mMeshes[i];
								for (physx::PxU32 i=0; i<m->mSubMeshCount; i++)
								{
									physx::SubMesh *sm = m->mSubMeshes[i];
									for (physx::PxU32 i=0; i<sm->mTriCount; i++)
									{
										physx::PxU32 i1 = sm->mIndices[i*3+0];
										physx::PxU32 i2 = sm->mIndices[i*3+1];
										physx::PxU32 i3 = sm->mIndices[i*3+2];
										physx::MeshVertex &v1 = m->mVertices[i1];
										physx::MeshVertex &v2 = m->mVertices[i2];
										physx::MeshVertex &v3 = m->mVertices[i3];
										bool newPos;
										i1 = vertices->getIndex(v1.mPos,newPos);
										i2 = vertices->getIndex(v2.mPos,newPos);
										i3 = vertices->getIndex(v3.mPos,newPos);
										indices.push_back(i1);
										indices.push_back(i2);
										indices.push_back(i3);
									}
								}
							}
						}
						else
						{
							printf("Failed to process the input file '%s'\r\n", wavefront );
						}
					}
					else
					{
						fclose(fph);
						printf("Can't process empty file.\r\n");
					}
					if ( indices.size() )
					{
						desc.mTriangleCount = (physx::PxU32)(indices.size()/3);
						desc.mIndices = &indices[0];
						desc.mVertexCount = vertices->getVcount();
						desc.mVertices = vertices->getVerticesFloat();
					}
				}
				else
				{
					printf("Failed to open input file '%s'\r\n", wavefront );
				}
			}
			else
			{
				printf("Failed to load MeshImporter DLL's?\r\n");
			}

#else
			WavefrontObj obj;
			unsigned int tcount = obj.loadObj(wavefront,false);
			if ( tcount )
			{
				desc.mTriangleCount = obj.mTriCount;
				desc.mVertexCount = obj.mVertexCount;
				desc.mIndices = (physx::PxU32 *)obj.mIndices;
				desc.mVertices = obj.mVertices;
			}
#endif		
			if ( desc.mTriangleCount )
			{
				physx::PxU32 hullCount = HACD::gHACD->performHACD(desc);
				if ( hullCount != 0 )
				{
					printf("Produced %d output convex hulls.\r\n", hullCount );
					FILE *fph = fopen("ConvexDecomposition.obj", "wb");
					if ( fph )
					{
						fprintf(fph,"# Input mesh '%s' produced %d convex hulls.\r\n", wavefront, hullCount );
						physx::PxU32 *baseVertex = new physx::PxU32[hullCount];
						physx::PxU32 vertexCount = 0;
						for (physx::PxU32 i=0; i<hullCount; i++)
						{
							const HACD::HACD_API::Hull *hull = HACD::gHACD->getHull(i);
							if ( hull )
							{
								baseVertex[i] = vertexCount;
								fprintf(fph,"## Hull %d has %d vertices.\r\n", i+1, hull->mVertexCount );
								for (physx::PxU32 i=0; i<hull->mVertexCount; i++)
								{
									const physx::PxF32 *p = &hull->mVertices[i*3];
									fprintf(fph,"v %0.9f %0.9f %0.9f\r\n", p[0], p[1], p[2] );
								}
								vertexCount+=hull->mVertexCount;
							}
						}
						for (physx::PxU32 i=0; i<hullCount; i++)
						{
							const HACD::HACD_API::Hull *hull = HACD::gHACD->getHull(i);
							if ( hull )
							{
								physx::PxU32 startVertex = baseVertex[i];
								fprintf(fph,"# Convex Hull %d contains %d triangles and %d vertices.  Starting vertex index is: %d It has a volume of: %0.9f\r\n", i+1, hull->mTriangleCount, hull->mVertexCount, startVertex);
								for (physx::PxU32 j=0; j<hull->mTriangleCount; j++)
								{
									physx::PxU32 i1 = hull->mIndices[j*3+0]+startVertex+1;
									physx::PxU32 i2 = hull->mIndices[j*3+1]+startVertex+1;
									physx::PxU32 i3 = hull->mIndices[j*3+2]+startVertex+1;
									fprintf(fph,"f %d %d %d\r\n", i1, i2, i3 );
								}
							}
						}

					}
					else
					{
						printf("Failed to open output file.\r\n");
					}
				}
			}
			else
			{
				printf("Failed to load Wavefront OBJ file '%s'\r\n",wavefront);
			}

#if USE_MESH_IMPORT
			if ( vertices )
			{
				fm_releaseVertexIndex(vertices);
			}
#endif
		}
		else
		{
			printf("Failed to load the HACD DLL\r\n");
		}
	}
}
