#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <conio.h>

#pragma warning(disable:4996 4100)

#include "PlatformConfig.h"
#include "wavefront.h"
#include "HACD.h"
#include "FloatMath.h"

#ifdef WIN32
#define USE_MESH_IMPORT 0
#endif

#if USE_MESH_IMPORT
#include "MeshImport.h"
#include "windows.h"

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


#endif

namespace HACD
{
	HACD_API *gHACD = NULL;
};


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
#if USE_MESH_IMPORT

void createSkeletalMesh(hacd::MeshImport *meshImport,hacd::MeshSystemContainer *msc,HACD::HACD_API *skel)
{
	hacd::MeshSystemContainer *output = meshImport->createMeshSystemContainer();
//	hacd::MeshSystem *ms = meshImport->getMeshSystem(msc);
	hacd::HaU32 boneCount = skel->getConstraintCount();
	if ( boneCount )
	{
		hacd::MeshImportInterface *import = meshImport->getMeshImportInterface(output);
		char **boneNames = (char **)PX_ALLOC(sizeof(char *)*boneCount);
		for (hacd::HaU32 i=0; i<boneCount; i++)
		{
			char scratch[512];
			sprintf_s(scratch,"bone%d", i );
			hacd::HaU32 slen = (hacd::HaU32)strlen(scratch);
			boneNames[i] = (char *)PX_ALLOC(slen+1);
			memcpy(boneNames[i],scratch,slen+1);
		}
		hacd::MeshSkeleton skeleton;
		skeleton.mBoneCount = boneCount;
		skeleton.mBones		= (hacd::MeshBone *)PX_ALLOC(sizeof(hacd::MeshBone)*boneCount);
		for (hacd::HaU32 i=0; i<boneCount; i++)
		{
			const HACD::HACD_API::Constraint *c = skel->getConstraint(i);
			hacd::MeshBone &b = skeleton.mBones[i];
			new ( &b ) hacd::MeshBone;
			b.mName = boneNames[i];
			b.mPosition[0] = c->mIntersect[0];
			b.mPosition[1] = c->mIntersect[1];
			b.mPosition[2] = c->mIntersect[2];
			b.mParentIndex = -1;
		}

		import->importSkeleton(skeleton);

		meshImport->gather(output);
		hacd::MeshSystem *out = meshImport->getMeshSystem(output);
		hacd::MeshSerialize mserialize(hacd::MSF_EZMESH);
		meshImport->serializeMeshSystem(out,mserialize);
		if ( mserialize.mBaseData )
		{
			FILE *fph = fopen("skeleton.ezm", "wb");
			fwrite(mserialize.mBaseData,mserialize.mBaseLen,1,fph);
			fclose(fph);
		}

		meshImport->releaseSerializeMemory(mserialize);

		PX_FREE(skeleton.mBones);
		PX_FREE(boneNames);
	}
	meshImport->releaseMeshSystemContainer(output);
}

#endif

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
		printf("-connect : The connection distance to use (merges discrete sub-meshes).  Default is zero.\r\n");
		printf("-constraint : Auto-generates constraints for the output convex hulls.\r\n");
		printf("-mesh	: Generates an output skeletal mesh with the mesh deformation distance passed. (Not yet implemented)\r\n");
		printf("\r\n");
		printf("Example: TestHACD hornbug.obj -c 500 -m 5 -connect 10\r\n");
		printf("\r\n");
	}
	else
	{
		HACD::HACD_API::Desc desc;
		const char *wavefront = argv[1];
		int scan = 2;
		bool generateConstraints = false;
		bool generateSkeletalMesh = false;
		hacd::HaF32 weightingDistance = 0.1f;
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
			else if ( strcmp(option,"-connect") == 0 )
			{
				desc.mConnectDistance = getFloatArg(scan+1,argc,argv);
				scan+=2;
			}
			else if ( strcmp(option,"-m") == 0 )
			{
				desc.mMinHullCount = getIntArg(scan+1,argc,argv);
				scan+=2;
			}
			else if ( strcmp(option,"-constraint") == 0 )
			{
				generateConstraints = true;
				scan++;
			}
			else if ( strcmp(option,"-mesh") == 0 )
			{
				weightingDistance = getFloatArg(scan+1,argc,argv);
				generateConstraints = true;
				generateSkeletalMesh = true;
				scan+=2;
			}
			else
			{
				scan++;
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
			hacd::MeshImport *meshImport = hacd::loadMeshImporters(path);
			hacd::MeshSystemContainer *msc = NULL;
			STDNAME::vector< hacd::HaU32 > indices;
			hacd::fm_VertexIndex *vertices = NULL;
			if ( meshImport )
			{
				printf("Opening input mesh file '%s' for read access.\r\n", wavefront );
				FILE *fph = fopen(wavefront,"rb");
				if ( fph )
				{
					fseek(fph,0L,SEEK_END);
					hacd::HaU32 len = ftell(fph);
					fseek(fph,0L,SEEK_SET);
					if ( len )
					{
						void *buffer = PX_ALLOC(len);
						fread(buffer,len,1,fph);
						fclose(fph);
						msc = meshImport->createMeshSystemContainer(wavefront,buffer,len,NULL);
						if ( msc )
						{
							hacd::MeshSystem *ms = meshImport->getMeshSystem(msc);
							// ok, we new need to convert the input graphics mesh into a single indexed triangle list.
							vertices = hacd::fm_createVertexIndex(0.0001f,false);
							for (hacd::HaU32 i=0; i<ms->mMeshCount; i++)
							{
								hacd::Mesh *m = ms->mMeshes[i];
								for (hacd::HaU32 i=0; i<m->mSubMeshCount; i++)
								{
									hacd::SubMesh *sm = m->mSubMeshes[i];
									for (hacd::HaU32 i=0; i<sm->mTriCount; i++)
									{
										hacd::HaU32 i1 = sm->mIndices[i*3+0];
										hacd::HaU32 i2 = sm->mIndices[i*3+1];
										hacd::HaU32 i3 = sm->mIndices[i*3+2];
										hacd::MeshVertex &v1 = m->mVertices[i1];
										hacd::MeshVertex &v2 = m->mVertices[i2];
										hacd::MeshVertex &v3 = m->mVertices[i3];
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
						desc.mTriangleCount = (hacd::HaU32)(indices.size()/3);
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
				desc.mIndices = (hacd::HaU32 *)obj.mIndices;
				desc.mVertices = obj.mVertices;
			}
#endif		
			if ( desc.mTriangleCount )
			{
				hacd::HaU32 hullCount = HACD::gHACD->performHACD(desc);
				if ( hullCount != 0 )
				{
					printf("Produced %d output convex hulls.\r\n", hullCount );
					FILE *fph = fopen("ConvexDecomposition.obj", "wb");
					if ( fph )
					{
						fprintf(fph,"# Input mesh '%s' produced %d convex hulls.\r\n", wavefront, hullCount );
						hacd::HaU32 *baseVertex = new hacd::HaU32[hullCount];
						hacd::HaU32 vertexCount = 0;
						for (hacd::HaU32 i=0; i<hullCount; i++)
						{
							const HACD::HACD_API::Hull *hull = HACD::gHACD->getHull(i);
							if ( hull )
							{
								baseVertex[i] = vertexCount;
								fprintf(fph,"## Hull %d has %d vertices.\r\n", i+1, hull->mVertexCount );
								for (hacd::HaU32 i=0; i<hull->mVertexCount; i++)
								{
									const hacd::HaF32 *p = &hull->mVertices[i*3];
									fprintf(fph,"v %0.9f %0.9f %0.9f\r\n", p[0], p[1], p[2] );
								}
								vertexCount+=hull->mVertexCount;
							}
						}
						for (hacd::HaU32 i=0; i<hullCount; i++)
						{
							const HACD::HACD_API::Hull *hull = HACD::gHACD->getHull(i);
							if ( hull )
							{
								hacd::HaU32 startVertex = baseVertex[i];
								fprintf(fph,"# Convex Hull %d contains %d triangles and %d vertices.  Starting vertex index is: %d It has a volume of: %0.9f\r\n", i+1, hull->mTriangleCount, hull->mVertexCount, startVertex);
								for (hacd::HaU32 j=0; j<hull->mTriangleCount; j++)
								{
									hacd::HaU32 i1 = hull->mIndices[j*3+0]+startVertex+1;
									hacd::HaU32 i2 = hull->mIndices[j*3+1]+startVertex+1;
									hacd::HaU32 i3 = hull->mIndices[j*3+2]+startVertex+1;
									fprintf(fph,"f %d %d %d\r\n", i1, i2, i3 );
								}
							}
						}
					}
					else
					{
						printf("Failed to open output file.\r\n");
					}
					if ( generateConstraints && hullCount > 1 )
					{
						printf("Auto generating constraints between the convex hulls.\r\n");
						hacd::HaU32 constraintCount = HACD::gHACD->generateConstraints();
						printf("Generated %d constraints.\r\n", constraintCount );
						for (hacd::HaU32 i=0; i<constraintCount; i++)
						{
							const HACD::HACD_API::Constraint *c = HACD::gHACD->getConstraint(i);
							if ( c ) 
							{
								printf("Constraint%d : From Hull %d to Hull %d at location(%0.4f,%0.4f,%0.4f)\r\n", i+1,
										c->mFrom,
										c->mTo,
										c->mIntersect[0],
										c->mIntersect[1],
										c->mIntersect[2] );
							}
						}
#if USE_MESH_IMPORT
						if ( generateSkeletalMesh )
						{
							createSkeletalMesh(meshImport,msc,HACD::gHACD);
						}
#endif
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
