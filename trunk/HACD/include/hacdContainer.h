#ifndef HACD_CONTAINER_H

#define HACD_CONTAINER_H

#include "PxSimpleTypes.h"
#include "hacdVector.h"
#include "PsArray.h"
#include "PxMapSet.h"

namespace HACD
{
	//! priority queque element
	class GraphEdgePQ
	{
	public:
		//! Constructor
		//! @param name edge's id
		//! @param priority edge's priority
		GraphEdgePQ(physx::PxI32 name, physx::PxF64 priority)
		{
			m_name = name;
			m_priority = priority;
		}
		//! Destructor
		~GraphEdgePQ(void){}
	private:
		physx::PxI32									m_name;						//!< edge name
		physx::PxF64									m_priority;					//!< priority
		//! Operator < for GraphEdgePQ
		friend bool                                 operator<(const GraphEdgePQ & lhs, const GraphEdgePQ & rhs);
		//! Operator > for GraphEdgePQ
		friend bool                                 operator>(const GraphEdgePQ & lhs, const GraphEdgePQ & rhs);
		friend class HACD;
	};
	inline bool										operator<(const GraphEdgePQ & lhs, const GraphEdgePQ & rhs)
	{
		if (lhs.m_priority!=rhs.m_priority) return lhs.m_priority<rhs.m_priority;
		return lhs.m_name < rhs.m_name;
	}
	inline bool										operator>(const GraphEdgePQ & lhs, const GraphEdgePQ & rhs)
	{
		if (lhs.m_priority!=rhs.m_priority) return lhs.m_priority>rhs.m_priority;
		return lhs.m_name > rhs.m_name;
	}


}; // end of HACD namespace

namespace HACD
{

typedef physx::set<physx::PxI32> PxI32Set;
typedef physx::set<physx::PxU64> PxU64Set;

typedef physx::Array< physx::PxU32 > PxU32Vector;
typedef physx::Array< Vec3<physx::PxF64> > Vec3Vector;
typedef physx::Array< Vec3<physx::PxI32> > TriVector;
typedef physx::Array< physx::PxI32 > PxI32Vector;
typedef physx::Array< PxI32Set > PxI32SetVector;
typedef physx::Array<GraphEdgePQ> GraphEdgePQVector;




}; // end of HACD namespace


namespace HACD
{
	class TMMEdge;
	//!	Vertex data structure used in a triangular manifold mesh (TMM).
	class TMMVertex
	{
	public:
		TMMVertex(void);
		~TMMVertex(void);

	private:
		Vec3<physx::PxF64>										m_pos;
		physx::PxI32												m_name;
		physx::PxU32												m_id;
		CircularListElement<TMMEdge> *						m_duplicate;		// pointer to incident cone edge (or NULL)
		bool												m_onHull;
		bool												m_tag;
		TMMVertex(const TMMVertex & rhs);

		friend class HACD;            
		friend class ICHull;
		friend class TMMesh;
		friend class TMMTriangle;
		friend class TMMEdge;
	};

	//!	Triangle data structure used in a triangular manifold mesh (TMM).
	class TMMTriangle
	{
	public:
		TMMTriangle(void);
		~TMMTriangle(void);
	private:
		physx::PxU32												m_id;
		CircularListElement<TMMEdge> *						m_edges[3];
		CircularListElement<TMMVertex> *					m_vertices[3];
		PxI32Set										m_incidentPoints;
		bool												m_visible;

		TMMTriangle(const TMMTriangle & rhs);

		friend class HACD;
		friend class ICHull;
		friend class TMMesh;
		friend class TMMVertex;
		friend class TMMEdge;
	};



	//!	Edge data structure used in a triangular manifold mesh (TMM).
	class TMMEdge
	{		
	public:
		TMMEdge(void);
		~TMMEdge(void);
	private:
		physx::PxU32												m_id;
		CircularListElement<TMMTriangle> *					m_triangles[2];
		CircularListElement<TMMVertex> *					m_vertices[2];
		CircularListElement<TMMTriangle> *					m_newFace;


		TMMEdge(const TMMEdge & rhs);

		friend class HACD;
		friend class ICHull;
		friend class TMMTriangle;
		friend class TMMVertex;
		friend class TMMesh;
	};

	class Material
	{
	public:    
		Material(void);
		~Material(void){}        
		//    private:
		Vec3<physx::PxF64>                                            m_diffuseColor;
		physx::PxF64                                                  m_ambientIntensity;
		Vec3<physx::PxF64>                                            m_specularColor;
		Vec3<physx::PxF64>                                            m_emissiveColor;
		physx::PxF64                                                  m_shininess;
		physx::PxF64                                                  m_transparency;

		friend class TMMesh;
		friend class HACD;
	};


	//!	triangular manifold mesh data structure.
	class TMMesh
	{
	public:

		//! Returns the number of vertices>
		inline physx::PxU32										GetNVertices() const { return m_vertices.GetSize();}
		//! Returns the number of edges
		inline physx::PxU32										GetNEdges() const { return m_edges.GetSize();}
		//! Returns the number of triangles
		inline physx::PxU32										GetNTriangles() const { return m_triangles.GetSize();}
		//! Returns the vertices circular list
		inline const CircularList<TMMVertex> &              GetVertices() const { return m_vertices;}
		//! Returns the edges circular list
		inline const CircularList<TMMEdge> &				GetEdges() const { return m_edges;}
		//! Returns the triangles circular list
		inline const CircularList<TMMTriangle> &			GetTriangles() const { return m_triangles;}
		//! Returns the vertices circular list
		inline CircularList<TMMVertex> &                    GetVertices() { return m_vertices;}
		//! Returns the edges circular list
		inline CircularList<TMMEdge> &                      GetEdges() { return m_edges;}
		//! Returns the triangles circular list
		inline CircularList<TMMTriangle> &                  GetTriangles() { return m_triangles;}               
		//! Add vertex to the mesh
		CircularListElement<TMMVertex> *					AddVertex() {return m_vertices.Add();}
		//! Add vertex to the mesh
		CircularListElement<TMMEdge> *						AddEdge() {return m_edges.Add();}
		//! Add vertex to the mesh
		CircularListElement<TMMTriangle> *					AddTriangle() {return m_triangles.Add();}
		//! Print mesh information 
		//!
		void                                                GetIFS(Vec3<physx::PxF64> * const points, Vec3<physx::PxI32> * const triangles);
		//!  
		void												Clear();
		//!
		void                                                Copy(TMMesh & mesh);
		//!
		bool												CheckConsistancy();
		//!
		bool												Normalize();
		//!
		bool												Denormalize();
		//!	Constructor
		TMMesh(void);
		//! Destructor
		virtual												~TMMesh(void);

	private:
		CircularList<TMMVertex>								m_vertices;
		CircularList<TMMEdge>								m_edges;
		CircularList<TMMTriangle>							m_triangles;
		physx::PxF64												m_diag;						//>! length of the BB diagonal
		Vec3<physx::PxF64>										m_barycenter;				//>! barycenter of the mesh

		// not defined
		TMMesh(const TMMesh & rhs);
		friend class ICHull;
		friend class HACD;
	};

	class DPoint;
	class HACD;
	//!	Incremental Convex Hull algorithm (cf. http://maven.smith.edu/~orourke/books/ftp.html ).
	enum ICHullError
	{
		ICHullErrorOK = 0,
		ICHullErrorCoplanarPoints,
		ICHullErrorNoVolume,
		ICHullErrorInconsistent,
		ICHullErrorNotEnoughPoints
	};
	typedef physx::map<physx::PxI32, DPoint> DPointMap;
	typedef physx::Array<CircularListElement<TMMEdge> *> CircularListElementTMMEdgeVector;
	typedef physx::Array<CircularListElement<TMMTriangle> *> CircularListElementTMMTriangleVector;
	class ICHull : public physx::UserAllocated
	{
	public:
		//!
		bool												IsFlat() { return m_isFlat;}
		//! 
		DPointMap *							GetDistPoints() const { return m_distPoints;}
		//!
		void												SetDistPoints(DPointMap * distPoints) { m_distPoints = distPoints;}
		//! Returns the computed mesh
		TMMesh &                                            GetMesh() { return m_mesh;}
		//!	Add one point to the convex-hull    
		bool                                                AddPoint(const Vec3<physx::PxF64> & point) {return AddPoints(&point, 1);}
		//!	Add one point to the convex-hull    
		bool                                                AddPoint(const Vec3<physx::PxF64> & point, physx::PxI32 id);
		//!	Add points to the convex-hull
		bool                                                AddPoints(const Vec3<physx::PxF64> * points, physx::PxU32 nPoints);	
		bool												AddPoints(Vec3Vector points);
		//!	
		ICHullError                                         Process();
		//! 
		ICHullError                                         Process(physx::PxU32 nPointsCH);
		//!
		physx::PxF64                                              ComputeVolume();
		//!
		bool                                                IsInside(const Vec3<physx::PxF64> pt);
		//!
		physx::PxF64												ComputeDistance(physx::PxI32 name, const Vec3<physx::PxF64> & pt, const Vec3<physx::PxF64> & normal, bool & insideHull, bool updateIncidentPoints);
		//!
		const ICHull &                                      operator=(ICHull & rhs);        

		//!	Constructor
		ICHull(void);
		//! Destructor
		virtual                                             ~ICHull(void) {};

	private:
		//!	DoubleTriangle builds the initial physx::PxF64 triangle.  It first finds 3 noncollinear points and makes two faces out of them, in opposite order. It then finds a fourth point that is not coplanar with that face.  The vertices are stored in the face structure in counterclockwise order so that the volume between the face and the point is negative. Lastly, the 3 newfaces to the fourth point are constructed and the data structures are cleaned up. 
		ICHullError                                         DoubleTriangle();
		//!	MakeFace creates a new face structure from three vertices (in ccw order).  It returns a pointer to the face.
		CircularListElement<TMMTriangle> *                  MakeFace(CircularListElement<TMMVertex> * v0,  
			CircularListElement<TMMVertex> * v1,
			CircularListElement<TMMVertex> * v2,
			CircularListElement<TMMTriangle> * fold);			
		//!	
		CircularListElement<TMMTriangle> *                  MakeConeFace(CircularListElement<TMMEdge> * e, CircularListElement<TMMVertex> * v);
		//!	
		bool                                                ProcessPoint();
		//!
		bool                                                ComputePointVolume(physx::PxF64 &totalVolume, bool markVisibleFaces);
		//!
		bool                                                FindMaxVolumePoint();
		//!	
		bool                                                CleanEdges();
		//!	
		bool                                                CleanVertices(physx::PxU32 & addedPoints);
		//!	
		bool                                                CleanTriangles();
		//!	
		bool                                                CleanUp(physx::PxU32 & addedPoints);
		//!
		bool                                                MakeCCW(CircularListElement<TMMTriangle> * f,
			CircularListElement<TMMEdge> * e, 
			CircularListElement<TMMVertex> * v);
		void												Clear(); 
	private:
		static const physx::PxI32                                   sc_dummyIndex;
		static const physx::PxF64                                 sc_distMin;
		TMMesh                                              m_mesh;
		CircularListElementTMMEdgeVector         m_edgesToDelete;
		CircularListElementTMMEdgeVector         m_edgesToUpdate;
		CircularListElementTMMTriangleVector     m_trianglesToDelete; 
		DPointMap *							m_distPoints;            
		CircularListElement<TMMVertex> *					m_dummyVertex;
		Vec3<physx::PxF64>										m_normal;
		bool												m_isFlat;


		ICHull(const ICHull & rhs);

		friend class HACD;
	};

	class DPoint  
	{
	public:       
		DPoint(physx::PxF64 dist=0.0, bool computed=false, bool distOnly=false)
			:m_dist(dist),
			m_computed(computed),
			m_distOnly(distOnly){};
		~DPoint(){};      
	private:
		physx::PxF64													m_dist;
		bool													m_computed;
		bool                                                    m_distOnly;
		friend class TMMTriangle;
		friend class TMMesh;
		friend class GraphVertex;
		friend class GraphEdge;
		friend class Graph;
		friend class ICHull;
		friend class HACD;
	};


	class GraphVertex  
	{
	public:
		bool                                    AddEdge(physx::PxI32 name) 
		{ 
			m_edges.insert(name); 
			return true; 
		}
		bool                                    DeleteEdge(physx::PxI32 name);        
		GraphVertex();
		~GraphVertex(){ delete m_convexHull;};      
	private:
		physx::PxI32                                    m_name;
		PxI32Set                          m_edges;
		physx::PxI32                                    m_cc;        
		bool                                    m_deleted;
		PxU32Vector	                    m_ancestors;
		DPointMap					m_distPoints;

		physx::PxF64                                  m_error;
		physx::PxF64                                  m_surf;
		physx::PxF64                                  m_volume;
		physx::PxF64                                  m_perimeter;
		physx::PxF64                                  m_concavity;
		ICHull *                                m_convexHull;
		PxU64Set			m_boudaryEdges;


		friend class GraphEdge;
		friend class Graph;
		friend class HACD;
	};

	class GraphEdge 
	{
	public:
		GraphEdge();
		~GraphEdge(){delete m_convexHull;};
	private:
		physx::PxI32                                    m_name;
		physx::PxI32                                    m_v1;
		physx::PxI32                                    m_v2;
		DPointMap					m_distPoints;
		physx::PxF64                                  m_error;
		physx::PxF64                                  m_surf;
		physx::PxF64                                  m_volume;
		physx::PxF64                                  m_perimeter;
		physx::PxF64                                  m_concavity;
		ICHull *                                m_convexHull;
		PxU64Set			m_boudaryEdges;
		bool                                    m_deleted;



		friend class GraphVertex;
		friend class Graph;
		friend class HACD;
	};

	typedef physx::Array<GraphEdge> GraphEdgeVector;
	typedef physx::Array<GraphVertex> GraphVertexVector;

}; // end of HACD namespace

#endif
