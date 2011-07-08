/* Copyright (c) 2011 Khaled Mamou (kmamou at gmail dot com)
 All rights reserved.
 
 
 Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 
 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 
 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 
 3. The names of the contributors may not be used to endorse or promote products derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "hacdGraph.h"

#pragma warning(disable:4702)

namespace HACD
{    
    
    GraphEdge::GraphEdge()
    {
        m_convexHull = 0;
        m_v1 = -1;
        m_v2 = -1;
		m_name = -1;
        m_error = 0.0;
        m_surf = 0.0;
        m_perimeter = 0.0;
        m_concavity = 0.0;
		m_volume = 0.0;
        m_deleted = false;
	}
   
    GraphVertex::GraphVertex()
    {
        m_convexHull = 0;
		m_name = -1;
        m_cc = -1;        
        m_error = 0.0;
        m_surf = 0.0;
        m_perimeter = 0.0;
        m_concavity = 0.0;
		m_volume = 0.0;
        m_deleted = false;
    }
    
    bool GraphVertex::DeleteEdge(physx::PxI32 name)
    {
        PxI32Set::iterator it = m_edges.find(name);
        if (it != m_edges.end() )
		{
			m_edges.erase(it);
			return true;
		}
        return false;
    }

    Graph::Graph()
    {
        m_nV = 0;
        m_nE = 0;
        m_nCCs = 0;
    }
    
    Graph::~Graph()
    {
    }
    
	void Graph::Allocate(physx::PxU32 nV, physx::PxU32 nE)
	{ 
		m_nV = nV;
		m_edges.reserve(nE);
		m_vertices.resize(nV);
		for(physx::PxU32 i = 0; i < nV; i++)
		{
			m_vertices[i].m_name = static_cast<physx::PxI32>(i);
		}
	}

    physx::PxI32 Graph::AddVertex()
    {
		physx::PxU32 name = (physx::PxU32)m_vertices.size();
		m_vertices.resize(name+1);
        m_vertices[name].m_name = name;
        m_nV++;
        return static_cast<physx::PxI32>(name);
    }
    
    physx::PxI32 Graph::AddEdge(physx::PxI32 v1, physx::PxI32 v2)
    {
		physx::PxU32 name =(physx::PxU32) m_edges.size();
		m_edges.push_back(GraphEdge());
        m_edges[name].m_name = name;
        m_edges[name].m_v1 = v1;
        m_edges[name].m_v2 = v2;
        m_vertices[v1].AddEdge(name);
        m_vertices[v2].AddEdge(name);
        m_nE++;
		return static_cast<physx::PxI32>(name);
    }

    bool Graph::DeleteEdge(physx::PxI32 name)
    {
		if (name < static_cast<physx::PxI32>(m_edges.size()))
		{
            physx::PxI32 v1 = m_edges[name].m_v1;
            physx::PxI32 v2 = m_edges[name].m_v2;
			m_edges[name].m_deleted = true;
            m_vertices[v1].DeleteEdge(name);
            m_vertices[v2].DeleteEdge(name);
            delete m_edges[name].m_convexHull;
			m_edges[name].m_distPoints.clear();
			m_edges[name].m_boudaryEdges.clear();
            m_edges[name].m_convexHull = 0;
			m_nE--;
			return true;
		}
		return false;
    }
    bool Graph::DeleteVertex(physx::PxI32 name)
    {
		if (name < static_cast<physx::PxI32>(m_vertices.size()))
		{
			m_vertices[name].m_deleted = true;
            m_vertices[name].m_edges.clear();
            m_vertices[name].m_ancestors = PxU32Vector();
            delete m_vertices[name].m_convexHull;
			m_vertices[name].m_distPoints.clear();
			m_vertices[name].m_boudaryEdges.clear();
            m_vertices[name].m_convexHull = 0;
			m_nV--;
			return true;
		}
		return false;
    }    
    bool Graph::EdgeCollapse(physx::PxI32 v1, physx::PxI32 v2)
	{
		physx::PxI32 edgeToDelete = GetEdgeID(v1, v2);
        if (edgeToDelete >= 0) 
		{
			// delete the edge (v1, v2)
			DeleteEdge(edgeToDelete);
			// add v2 to v1 ancestors
            m_vertices[v1].m_ancestors.push_back(v2);
			// add v2's ancestors to v1's ancestors
			PxU32Vector newArray;
			PxU32Vector	&oldArray = m_vertices[v1].m_ancestors;
			PxU32Vector	&insertArray = m_vertices[v2].m_ancestors;
			newArray.reserve( insertArray.size() + oldArray.size() );
			for (physx::PxU32 i=0; i<insertArray.size(); i++)
			{
				newArray.push_back( insertArray[i] );
			}
			for (physx::PxU32 i=0; i<oldArray.size(); i++)
			{
				newArray.push_back( oldArray[i] );
			}
			m_vertices[v1].m_ancestors = newArray;

			// update adjacency information
			PxI32Set & v1Edges =  m_vertices[v1].m_edges;
			PxI32Set::const_iterator ed(m_vertices[v2].m_edges.begin());
			PxI32Set::const_iterator itEnd(m_vertices[v2].m_edges.end());
			physx::PxI32 b = -1;
			for(; ed != itEnd; ++ed) 
			{
				if (m_edges[*ed].m_v1 == v2)
				{
					b = m_edges[*ed].m_v2;
				}
				else
				{
					b = m_edges[*ed].m_v1;
				}
				if (GetEdgeID(v1, b) >= 0)
				{
					m_edges[*ed].m_deleted = true;
					m_vertices[b].DeleteEdge(*ed);
					m_nE--;
				}
				else
				{
					m_edges[*ed].m_v1 = v1;
					m_edges[*ed].m_v2 = b;
					v1Edges.insert(*ed);
				}
			}
			// delete the vertex v2
            DeleteVertex(v2);			
            return true;
        }
		return false;
    }
    
    physx::PxI32 Graph::GetEdgeID(physx::PxI32 v1, physx::PxI32 v2) const
    {
		if (v1 < static_cast<physx::PxI32>(m_vertices.size()) && !m_vertices[v1].m_deleted)
		{
			PxI32Set::const_iterator ed(m_vertices[v1].m_edges.begin());
			PxI32Set::const_iterator itEnd(m_vertices[v1].m_edges.end());
			for(; ed != itEnd; ++ed) 
			{
				if ( (m_edges[*ed].m_v1 == v2) || 
					 (m_edges[*ed].m_v2 == v2)   ) 
				{
					return m_edges[*ed].m_name;
				}
			}
		}
        return -1;
    }
    
    physx::PxI32 Graph::ExtractCCs()
	{
        // all CCs to -1
        for (physx::PxU32 v = 0; v < m_vertices.size(); ++v) 
		{
			if (!m_vertices[v].m_deleted)
			{
				m_vertices[v].m_cc = -1;
			}
        }
        
        // we get the CCs
        m_nCCs = 0;
		physx::PxI32 v2 = -1;
		PxU32Vector temp;
        for (physx::PxU32 v = 0; v < m_vertices.size(); ++v) 
		{
			if (!m_vertices[v].m_deleted && m_vertices[v].m_cc == -1) 
			{
                m_vertices[v].m_cc = m_nCCs;
                temp.clear();
                temp.push_back(m_vertices[v].m_name);
                while (temp.size()) 
				{
                    physx::PxI32 vertex = temp[temp.size()-1];
                    temp.pop_back();                    
					PxI32Set::const_iterator ed(m_vertices[vertex].m_edges.begin());
					PxI32Set::const_iterator itEnd(m_vertices[vertex].m_edges.end());
					for(; ed != itEnd; ++ed) 
					{
                        if (m_edges[*ed].m_v1 == vertex) 
						{
                            v2 = m_edges[*ed].m_v2;
                        }
                        else 
						{
                            v2 = m_edges[*ed].m_v1;
                        }
                        if ( !m_vertices[v2].m_deleted && m_vertices[v2].m_cc == -1) 
						{
                            m_vertices[v2].m_cc = m_nCCs;
                            temp.push_back(v2);
                        }
                    }
                }
                m_nCCs++;
            }
        }        
        return m_nCCs;
    }
    void Graph::ConnectCCs()
	{
		return;
        ExtractCCs();
		PxU32Vector clustersRep;
		clustersRep.resize(m_nCCs,(physx::PxU32) -1);
		physx::PxU32 done = 0;
        for (physx::PxU32 v = 0; (done<m_nCCs) && (v < m_vertices.size()); ++v) 
		{
			if (!m_vertices[v].m_deleted && m_vertices[v].m_cc >=0 && clustersRep[m_vertices[v].m_cc] == -1)
			{
				clustersRep[m_vertices[v].m_cc] = static_cast<physx::PxI32>(done);
				done++;
			}
        }
        for (physx::PxU32 i = 0; i < m_nCCs; i++) 
		{
            for (physx::PxU32 j = i+1; j < m_nCCs; j++) 
			{
                AddEdge(clustersRep[i], clustersRep[j]);
            }
		}
    }
	void Graph::Clear()
	{
		m_vertices.clear();
		m_edges.clear();
		m_nV = 0;
        m_nE = 0;
        m_nCCs = 0;
	}

}