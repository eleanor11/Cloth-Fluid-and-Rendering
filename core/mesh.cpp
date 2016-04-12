// This code contains NVIDIA Confidential Information and is disclosed to you
// under a form of NVIDIA software license agreement provided separately to you.
//
// Notice
// NVIDIA Corporation and its licensors retain all intellectual property and
// proprietary rights in and to this software and related documentation and
// any modifications thereto. Any use, reproduction, disclosure, or
// distribution of this software and related documentation without an express
// license agreement from NVIDIA Corporation is strictly prohibited.
//
// ALL NVIDIA DESIGN SPECIFICATIONS, CODE ARE PROVIDED "AS IS.". NVIDIA MAKES
// NO WARRANTIES, EXPRESSED, IMPLIED, STATUTORY, OR OTHERWISE WITH RESPECT TO
// THE MATERIALS, AND EXPRESSLY DISCLAIMS ALL IMPLIED WARRANTIES OF NONINFRINGEMENT,
// MERCHANTABILITY, AND FITNESS FOR A PARTICULAR PURPOSE.
//
// Information and code furnished is believed to be accurate and reliable.
// However, NVIDIA Corporation assumes no responsibility for the consequences of use of such
// information or for any infringement of patents or other rights of third parties that may
// result from its use. No license is granted by implication or otherwise under any patent
// or patent rights of NVIDIA Corporation. Details are subject to change without notice.
// This code supersedes and replaces all information previously supplied.
// NVIDIA Corporation products are not authorized for use as critical
// components in life support devices or systems without express written approval of
// NVIDIA Corporation.
//
// Copyright (c) 2013-2015 NVIDIA Corporation. All rights reserved.
#include "mesh.h"
#include "platform.h"

#include <map>
#include <fstream>
#include <iostream>

using namespace std;

void Mesh::DuplicateVertex(uint32_t i)
{
	assert(m_positions.size() > i);	
	m_positions.push_back(m_positions[i]);
	
	if (m_normals.size() > i)
		m_normals.push_back(m_normals[i]);
	
	if (m_colours.size() > i)
		m_colours.push_back(m_colours[i]);
	
	if (m_texcoords[0].size() > i)
		m_texcoords[0].push_back(m_texcoords[0][i]);
	
	if (m_texcoords[1].size() > i)
		m_texcoords[1].push_back(m_texcoords[1][i]);
	
}

void Mesh::Normalize(float s)
{
	Vec3 lower, upper;
	GetBounds(lower, upper);
	Vec3 edges = upper-lower;

	Transform(TranslationMatrix(Point3(-lower)));

	float maxEdge = max(edges.x, max(edges.y, edges.z));
	Transform(ScaleMatrix(s/maxEdge));
}

void Mesh::CalculateNormals()
{
	m_normals.resize(0);
	m_normals.resize(m_positions.size());

	int numTris = int(GetNumFaces());

	for (int i=0; i < numTris; ++i)
	{
		int a = m_indices[i*3+0];
		int b = m_indices[i*3+1];
		int c = m_indices[i*3+2];
		
		Vec3 n = Cross(m_positions[b]-m_positions[a], m_positions[c]-m_positions[a]);

		m_normals[a] += n;
		m_normals[b] += n;
		m_normals[c] += n;
	}

	int numVertices = int(GetNumVertices());

	for (int i=0; i < numVertices; ++i)
		m_normals[i] = ::Normalize(m_normals[i]);
}

namespace 
{

    enum PlyFormat
    {
        eAscii,
        eBinaryBigEndian    
    };

    template <typename T>
    T PlyRead(ifstream& s, PlyFormat format)
    {
        T data = eAscii;

        switch (format)
        {
            case eAscii:
            {
                s >> data;
                break;
            }
            case eBinaryBigEndian:
            {
                char c[sizeof(T)];
                s.read(c, sizeof(T));
                reverse(c, c+sizeof(T));
                data = *(T*)c;
                break;
            }      
			default:
				assert(0);
        }

        return data;
    }

} // namespace anonymous


Mesh* ImportMesh(const char* path)
{
	std::string ext = GetExtension(path);

	Mesh* mesh = NULL;

	if (ext == "ply")
		mesh = ImportMeshFromPly(path);
	else if (ext == "obj")
		mesh = ImportMeshFromObj(path);


	return mesh;
}

/*added by eleanor*/

Vec3 sortips(int a, int b, int c) {
	int tmp;
	if (a > b) {
		tmp = a; a = b; b = tmp;
	}
	if (a > c) {
		tmp = a; a = c; c = tmp;
	}
	if (b > c) {
		tmp = b; b = c; c = tmp;
	}
	return Vec3(a, b, c);
}
void addTriangleToPoints(Mesh* mesh, int ip, int it) {
	int idx = mesh->m_pointTriangleNums[ip];
	Vec4 tmp1 = mesh->m_pointTriangles[ip * 2];
	Vec4 tmp2 = mesh->m_pointTriangles[ip * 2 + 1];

	switch (idx)
	{
	case 0:
		tmp1.w = it;
		break;
	case 1:
		tmp1.x = it;
		break;
	case 2:
		tmp1.y = it;
		break;
	case 3: 
		tmp1.z = it;
		break;
	case 4:
		tmp2.w = it;
		break;
	case 5:
		tmp2.x = it;
		break;
	case 6:
		tmp2.y = it;
		break;
	case 7:
		tmp2.z = it;
		break;
	default:
		break;
	}

	mesh->m_pointTriangleNums[ip] = idx + 1;
	mesh->m_pointTriangles[ip * 2] = tmp1;
	mesh->m_pointTriangles[ip * 2 + 1] = tmp2;
}
void renewTrianglePoints(Mesh* mesh, int it, int ip0, int ip1, int ip2) {
	mesh->m_trianglePoints[it] = sortips(ip0, ip1, ip2);
	addTriangleToPoints(mesh, ip0, it);
	addTriangleToPoints(mesh, ip1, it);
	addTriangleToPoints(mesh, ip2, it);
}

/*add end*/

Mesh* ImportMeshFromPly(const char* path)
{
    ifstream file(path, ios_base::in | ios_base::binary);

    if (!file)
        return NULL;

    // some scratch memory
    const uint32_t kMaxLineLength = 1024;
    char buffer[kMaxLineLength];

    //double startTime = GetSeconds();

    file >> buffer;
    if (strcmp(buffer, "ply") != 0)
        return NULL;

    PlyFormat format = eAscii;

    uint32_t numFaces = 0;
    uint32_t numVertices = 0;

    const uint32_t kMaxProperties = 16;
    uint32_t numProperties = 0; 
    float properties[kMaxProperties];

    bool vertexElement = false;

    while (file)
    {
        file >> buffer;

        if (strcmp(buffer, "element") == 0)
        {
            file >> buffer;

            if (strcmp(buffer, "face") == 0)
            {                
                vertexElement = false;
                file >> numFaces;
            }

            else if (strcmp(buffer, "vertex") == 0)
            {
                vertexElement = true;
                file >> numVertices;
            }
        }
        else if (strcmp(buffer, "format") == 0)
        {
            file >> buffer;
            if (strcmp(buffer, "ascii") == 0)
            {
                format = eAscii;
            }
            else if (strcmp(buffer, "binary_big_endian") == 0)
            {
                format = eBinaryBigEndian;
            }
			else
			{
				printf("Ply: unknown format\n");
				return false;
			}
        }
        else if (strcmp(buffer, "property") == 0)
        {
            if (vertexElement)
                ++numProperties;
        }
        else if (strcmp(buffer, "end_header") == 0)
        {
            break;
        }
    }

    // eat newline
    char nl;
    file.read(&nl, 1);
	
	// debug
	printf ("Loaded mesh: %s numFaces: %d numVertices: %d format: %d numProperties: %d\n", path, numFaces, numVertices, format, numProperties);

    Mesh* mesh = new Mesh;

    mesh->m_positions.resize(numVertices);
    mesh->m_normals.resize(numVertices);
    mesh->m_colours.resize(numVertices, Colour(1.0f, 1.0f, 1.0f, 1.0f));

    mesh->m_indices.reserve(numFaces*3);

	/*added by eleanor*/

	mesh->m_pointTriangleNums.resize(numVertices);
	mesh->m_pointTriangles.resize(numVertices * 2);
	mesh->m_trianglePoints.resize(numFaces);

	/*add end*/

    // read vertices
    for (uint32_t v=0; v < numVertices; ++v)
    {
        for (uint32_t i=0; i < numProperties; ++i)
        {
            properties[i] = PlyRead<float>(file, format);
        }

        mesh->m_positions[v] = Point3(properties[0], properties[1], properties[2]);
        mesh->m_normals[v] = Vector3(0.0f, 0.0f, 0.0f);

		/*added by eleanor*/
		mesh->m_pointTriangleNums[v] = 0;
		mesh->m_pointTriangles[v * 2] = Vec4(-1.0, -1.0, -1.0, -1.0);
		mesh->m_pointTriangles[v * 2 + 1] = Vec4(-1.0, -1.0, -1.0, -1.0);
		/*add end*/

    }

	int idx = 0;

    // read indices
    for (uint32_t f=0; f < numFaces; ++f)
    {
        uint32_t numIndices = (format == eAscii)?PlyRead<uint32_t>(file, format):PlyRead<uint8_t>(file, format);
		uint32_t indices[4];

		for (uint32_t i=0; i < numIndices; ++i)
		{
			indices[i] = PlyRead<uint32_t>(file, format);
		}

		switch (numIndices)
		{
		case 3:
			mesh->m_indices.push_back(indices[0]);
			mesh->m_indices.push_back(indices[1]);
			mesh->m_indices.push_back(indices[2]);

			/*added by eleanor*/
			renewTrianglePoints(mesh, idx, indices[0], indices[1], indices[2]);
			idx++;
			/*add end*/

			break;
		case 4:
			mesh->m_indices.push_back(indices[0]);
			mesh->m_indices.push_back(indices[1]);
			mesh->m_indices.push_back(indices[2]);

			/*added by eleanor*/
			renewTrianglePoints(mesh, idx, indices[0], indices[1], indices[2]);
			idx++;
			/*add end*/

			mesh->m_indices.push_back(indices[2]);
			mesh->m_indices.push_back(indices[3]);
			mesh->m_indices.push_back(indices[0]);

			/*added by eleanor*/
			renewTrianglePoints(mesh, idx, indices[2], indices[3], indices[0]);
			idx++;
			/*add end*/

			break;

		default:
			assert(!"invalid number of indices, only support tris and quads");
			break;
		};

		// calculate vertex normals as we go
        Point3& v0 = mesh->m_positions[indices[0]];
        Point3& v1 = mesh->m_positions[indices[1]];
        Point3& v2 = mesh->m_positions[indices[2]];

        Vector3 n = SafeNormalize(Cross(v1-v0, v2-v0), Vector3(0.0f, 1.0f, 0.0f));

		for (uint32_t i=0; i < numIndices; ++i)
		{
	        mesh->m_normals[indices[i]] += n;
	    }
	}

    for (uint32_t i=0; i < numVertices; ++i)
    {
        mesh->m_normals[i] = SafeNormalize(mesh->m_normals[i], Vector3(0.0f, 1.0f, 0.0f));
    }

    //cout << "Imported mesh " << path << " in " << (GetSeconds()-startTime)*1000.f << "ms" << endl;

    return mesh;

}

// map of Material name to Material
struct VertexKey
{
	VertexKey() :  v(0), vt(0), vn(0) {}
	
	uint32_t v, vt, vn;
	
	bool operator == (const VertexKey& rhs) const
	{
		return v == rhs.v && vt == rhs.vt && vn == rhs.vn;
	}
	
	bool operator < (const VertexKey& rhs) const
	{
		if (v != rhs.v)
			return v < rhs.v;
		else if (vt != rhs.vt)
			return vt < rhs.vt;
		else
			return vn < rhs.vn;
	}
};

Mesh* ImportMeshFromObj(const char* path)
{
    ifstream file(path);

    if (!file)
        return NULL;

    Mesh* m = new Mesh();

    vector<Point3> positions;
    vector<Vector3> normals;
    vector<Vector2> texcoords;
    vector<Vector3> colors;
    vector<uint32_t>& indices = m->m_indices;

    //typedef unordered_map<VertexKey, uint32_t, MemoryHash<VertexKey> > VertexMap;
    typedef map<VertexKey, uint32_t> VertexMap;
    VertexMap vertexLookup;	

    // some scratch memory
    const uint32_t kMaxLineLength = 1024;
    char buffer[kMaxLineLength];

    //double startTime = GetSeconds();

    while (file)
    {
        file >> buffer;
	
        if (strcmp(buffer, "vn") == 0)
        {
            // normals
            float x, y, z;
            file >> x >> y >> z;

            normals.push_back(Vector3(x, y, z));
        }
        else if (strcmp(buffer, "vt") == 0)
        {
            // texture coords
            float u, v;
            file >> u >> v;

            texcoords.push_back(Vector2(u, v));
        }
        else if (buffer[0] == 'v')
        {
            // positions
            float x, y, z;
            file >> x >> y >> z;

            positions.push_back(Point3(x, y, z));
        }
        else if (buffer[0] == 's' || buffer[0] == 'g' || buffer[0] == 'o')
        {
            // ignore smoothing groups, groups and objects
            char linebuf[256];
            file.getline(linebuf, 256);		
        }
        else if (strcmp(buffer, "mtllib") == 0)
        {
            // ignored
            std::string MaterialFile;
            file >> MaterialFile;
        }		
        else if (strcmp(buffer, "usemtl") == 0)
        {
            // read Material name
            std::string materialName;
            file >> materialName;
        }
        else if (buffer[0] == 'f')
        {
            // faces
            uint32_t faceIndices[4];
            uint32_t faceIndexCount = 0;

            for (int i=0; i < 4; ++i)
            {
                VertexKey key;

                file >> key.v;

				if (!file.eof())
				{
					// failed to read another index continue on
					if (file.fail())
					{
						file.clear();
						break;
					}

					if (file.peek() == '/')
					{
						file.ignore();

						if (file.peek() != '/')
						{
							file >> key.vt;
						}

						if (file.peek() == '/')
						{
							file.ignore();
							file >> key.vn;
						}
					}

					// find / add vertex, index
					VertexMap::iterator iter = vertexLookup.find(key);

					if (iter != vertexLookup.end())
					{
						faceIndices[faceIndexCount++] = iter->second;
					}
					else
					{
						// add vertex
						uint32_t newIndex = uint32_t(m->m_positions.size());
						faceIndices[faceIndexCount++] = newIndex;

						vertexLookup.insert(make_pair(key, newIndex)); 	

						// push back vertex data
						assert(key.v > 0);

						m->m_positions.push_back(positions[key.v-1]);
						
						// obj format doesn't support mesh colours so add default value
						m->m_colours.push_back(Colour(1.0f, 1.0f, 1.0f));

						// normal [optional]
						if (key.vn)
						{
							m->m_normals.push_back(normals[key.vn-1]);
						}

						// texcoord [optional]
						if (key.vt)
						{
							m->m_texcoords[0].push_back(texcoords[key.vt-1]);
						}
					}
				}
            }

            if (faceIndexCount == 3)
            {
                // a triangle
                indices.insert(indices.end(), faceIndices, faceIndices+3);
            }
            else if (faceIndexCount == 4)
            {
                // a quad, triangulate clockwise
                indices.insert(indices.end(), faceIndices, faceIndices+3);

                indices.push_back(faceIndices[2]);
                indices.push_back(faceIndices[3]);
                indices.push_back(faceIndices[0]);
            }
            else
            {
                cout << "Face with more than 4 vertices are not supported" << endl;
            }

        }		
        else if (buffer[0] == '#')
        {
            // comment
            char linebuf[256];
            file.getline(linebuf, 256);
        }
    }

    // calculate normals if none specified in file
    m->m_normals.resize(m->m_positions.size());

    const uint32_t numFaces = uint32_t(indices.size())/3;
    for (uint32_t i=0; i < numFaces; ++i)
    {
        uint32_t a = indices[i*3+0];
        uint32_t b = indices[i*3+1];
        uint32_t c = indices[i*3+2];

        Point3& v0 = m->m_positions[a];
        Point3& v1 = m->m_positions[b];
        Point3& v2 = m->m_positions[c];

        Vector3 n = SafeNormalize(Cross(v1-v0, v2-v0), Vector3(0.0f, 1.0f, 0.0f));

        m->m_normals[a] += n;
        m->m_normals[b] += n;
        m->m_normals[c] += n;
    }

    for (uint32_t i=0; i < m->m_normals.size(); ++i)
    {
        m->m_normals[i] = SafeNormalize(m->m_normals[i], Vector3(0.0f, 1.0f, 0.0f));
    }
        
    //cout << "Imported mesh " << path << " in " << (GetSeconds()-startTime)*1000.f << "ms" << endl;

    return m;
}

void ExportToObj(const char* path, const Mesh& m)
{
	ofstream file(path);

    if (!file)
        return;

	file << "# positions" << endl;

	for (uint32_t i=0; i < m.m_positions.size(); ++i)
	{
		Point3 v = m.m_positions[i];
		file << "v " << v.x << " " << v.y << " " << v.z << endl;
	}

	file << "# texcoords" << endl;

	for (uint32_t i=0; i < m.m_texcoords[0].size(); ++i)
	{
		Vec2 t = m.m_texcoords[0][i];
		file << "vt " << t.x << " " << t.y << endl;
	}

	file << "# normals" << endl;

	for (uint32_t i=0; i < m.m_normals.size(); ++i)
	{
		Vec3 n = m.m_normals[0][i];
		file << "vn " << n.x << " " << n.y << " " << n.z << endl;
	}

	file << "# faces" << endl;

	for (uint32_t i=0; i < m.m_indices.size()/3; ++i)
	{
		uint32_t j = i+1;

		// no sharing, assumes there is a unique position, texcoord and normal for each vertex
		file << "f " << j << "/" << j << "/" << j << endl;
	}
}

void Mesh::AddMesh(Mesh& m)
{
    uint32_t offset = uint32_t(m_positions.size());

    // add new vertices
    m_positions.insert(m_positions.end(), m.m_positions.begin(), m.m_positions.end());
    m_normals.insert(m_normals.end(), m.m_normals.begin(), m.m_normals.end());
    m_colours.insert(m_colours.end(), m.m_colours.begin(), m.m_colours.end());

    // add new indices with offset
    for (uint32_t i=0; i < m.m_indices.size(); ++i)
    {
        m_indices.push_back(m.m_indices[i]+offset);
    }    
}


void Mesh::Transform(const Matrix44& m)
{
    for (uint32_t i=0; i < m_positions.size(); ++i)
    {
        m_positions[i] = m*m_positions[i];
        m_normals[i] = m*m_normals[i];
    }
}

void Mesh::GetBounds(Vector3& outMinExtents, Vector3& outMaxExtents) const
{
    Point3 minExtents(FLT_MAX);
    Point3 maxExtents(-FLT_MAX);

    // calculate face bounds
    for (uint32_t i=0; i < m_positions.size(); ++i)
    {
        const Point3& a = m_positions[i];

        minExtents = Min(a, minExtents);
        maxExtents = Max(a, maxExtents);
    }

    outMinExtents = Vector3(minExtents);
    outMaxExtents = Vector3(maxExtents);
}

Mesh* CreateQuadMesh(float size, float y)
{
    uint32_t indices[] = { 0, 1, 2, 2, 3, 0 };
    Point3 positions[4];
    Vector3 normals[4];

    positions[0] = Point3(-size, y, size);
    positions[1] = Point3(size, y, size);
    positions[2] = Point3(size, y, -size);
    positions[3] = Point3(-size, y, -size);
    
    normals[0] = Vector3(0.0f, 1.0f, 0.0f);
    normals[1] = Vector3(0.0f, 1.0f, 0.0f);
    normals[2] = Vector3(0.0f, 1.0f, 0.0f);
    normals[3] = Vector3(0.0f, 1.0f, 0.0f);

    Mesh* m = new Mesh();
    m->m_indices.insert(m->m_indices.begin(), indices, indices+6);
    m->m_positions.insert(m->m_positions.begin(), positions, positions+4);
    m->m_normals.insert(m->m_normals.begin(), normals, normals+4);

    return m;
}

Mesh* CreateDiscMesh(float radius, uint32_t segments)
{
	const uint32_t numVerts = 1 + segments;

	Mesh* m = new Mesh();
	m->m_positions.resize(numVerts);
	m->m_normals.resize(numVerts);

	m->m_positions[0] = Point3(0.0f);
	m->m_positions[1] = Point3(0.0f, 0.0f, radius);

	for (uint32_t i=1; i <= segments; ++i)
	{
		uint32_t nextVert = (i+1)%numVerts;

		if (nextVert == 0)
			nextVert = 1;
		
		m->m_positions[nextVert] = Point3(radius*Sin((float(i)/segments)*k2Pi), 0.0f, radius*Cos((float(i)/segments)*k2Pi));
		m->m_normals[nextVert] = Vector3(0.0f, 1.0f, 0.0f);

		m->m_indices.push_back(0);
		m->m_indices.push_back(i);
		m->m_indices.push_back(nextVert);		
	}
	
	return m;
}

Mesh* CreateTetrahedron()
{
	Mesh* m = new Mesh();

	const Point3 vertices[4] = 
	{
		Point3(-1.0f, 0.0f, -1.0f/sqrtf(2.0f)),
		Point3(1.0f, 0.0f, -1.0f/sqrtf(2.0f)),
		Point3(0.0f, 1.0f, 1.0f/sqrtf(2.0f)),
		Point3(0.0f, -1.0f, 1.0f/sqrtf(2.0f))
	};

	const int indices[12] = 
	{
		0, 1, 2,
		2, 1, 3,
		2, 0, 3,
		0, 3, 1
	};

	m->m_positions.assign(vertices, vertices+4);
	m->m_indices.assign(indices, indices+12);

	m->CalculateNormals();

	return m;
	
}



