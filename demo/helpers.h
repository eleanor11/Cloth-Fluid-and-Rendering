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
#pragma once

#include <stdarg.h>



float SampleSDF(const float* sdf, int dim, int x, int y, int z)
{
	assert(x < dim && x >= 0);
	assert(y < dim && y >= 0);
	assert(z < dim && z >= 0);

	return sdf[z*dim*dim + y*dim + x];
}

// return normal of signed distance field
Vec3 SampleSDFGrad(const float* sdf, int dim, int x, int y, int z)
{
	int x0 = max(x-1, 0);
	int x1 = min(x+1, dim-1);

	int y0 = max(y-1, 0);
	int y1 = min(y+1, dim-1);

	int z0 = max(z-1, 0);
	int z1 = min(z+1, dim-1);

	float dx = (SampleSDF(sdf, dim, x1, y, z) - SampleSDF(sdf, dim, x0, y, z))*(dim*0.5f);
	float dy = (SampleSDF(sdf, dim, x, y1, z) - SampleSDF(sdf, dim, x, y0, z))*(dim*0.5f);
	float dz = (SampleSDF(sdf, dim, x, y, z1) - SampleSDF(sdf, dim, x, y, z0))*(dim*0.5f);

	return Vec3(dx, dy, dz);
}

void GetParticleBounds(Vec3& lower, Vec3& upper)
{
	lower = Vec3(FLT_MAX);
	upper = Vec3(-FLT_MAX);

	for (size_t i=0; i < g_positions.size(); ++i)
	{
		lower = Min(Vec3(g_positions[i]), lower);
		upper = Max(Vec3(g_positions[i]), upper);
	}
}

void CreateParticleGrid(Vec3 lower, int dimx, int dimy, int dimz, float radius, Vec3 velocity, float invMass, bool rigid, float rigidStiffness, int phase, float jitter=0.005f)
{
	if (rigid && g_rigidIndices.empty())
		g_rigidOffsets.push_back(0);

	for (int x=0; x < dimx; ++x)
	{
		for (int y=0; y < dimy; ++y)
		{
			for (int z=0; z < dimz; ++z)
			{
				if (rigid)
					g_rigidIndices.push_back(int(g_positions.size()));

				Vec3 position = lower + Vec3(float(x), float(y), float(z))*radius + RandomUnitVector()*jitter;

				g_positions.push_back(Vec4(position.x, position.y, position.z, invMass));
				g_velocities.push_back(velocity);
				g_phases.push_back(phase);
			}
		}
	}

	if (rigid)
	{
		g_rigidCoefficients.push_back(rigidStiffness);
		g_rigidOffsets.push_back(int(g_rigidIndices.size()));
	}
}

void CreateParticleGrid2(Vec3 lower, int dimx, int dimy, int dimz, float radius, Vec3 velocity, float invMass, bool rigid, float rigidStiffness, int phase, float jitter = 0.005f)
{
	int activeCount = flexGetActiveCount(g_flex);
	for (int x = 0; x < dimx; ++x)
	{
		for (int y = 0; y < dimy; ++y)
		{
			for (int z = 0; z < dimz; ++z)
			{
				if (rigid)
					g_rigidIndices.push_back(int(g_positions.size()));

				Vec3 position = lower + Vec3(float(x), float(y), float(z))*radius + RandomUnitVector()*jitter;

				g_positions[activeCount] = Vec4(position.x, position.y, position.z, invMass);
				g_velocities[activeCount] = velocity;
				g_phases[activeCount] = phase;

				//g_positions.push_back(Vec4(position.x, position.y, position.z, invMass));
				//g_velocities.push_back(velocity);
				//g_phases.push_back(phase);

				activeCount++;
			}
		}
	}
	flexSetActive(g_flex, &g_activeIndices[0], activeCount, eFlexMemoryHost);

}

void CreateParticleSphere(Vec3 center, int dim, float radius, Vec3 velocity, float invMass, bool rigid, float rigidStiffness, int phase, float jitter=0.005f)
{
	if (rigid && g_rigidIndices.empty())
			g_rigidOffsets.push_back(0);

	for (int x=0; x <= dim; ++x)
	{
		for (int y=0; y <= dim; ++y)
		{
			for (int z=0; z <= dim; ++z)
			{
				float sx = x - dim*0.5f;
				float sy = y - dim*0.5f;
				float sz = z - dim*0.5f;

				if (sx*sx + sy*sy + sz*sz <= dim*dim/4)
				{
					if (rigid)
						g_rigidIndices.push_back(int(g_positions.size()));

					Vec3 position = center + radius*Vec3(sx, sy, sz) + RandomUnitVector()*jitter;

					g_positions.push_back(Vec4(position.x, position.y, position.z, invMass));
					g_velocities.push_back(velocity);
					g_phases.push_back(phase);
				}
			}
		}
	}

	if (rigid)
	{
		g_rigidCoefficients.push_back(rigidStiffness);
		g_rigidOffsets.push_back(int(g_rigidIndices.size()));
	}
}

void CreateParticleShape(const char* file, Vec3 lower, Vec3 scale, float rotation, float spacing, Vec3 velocity, float invMass, bool rigid, float rigidStiffness, int phase, bool skin, float jitter=0.005f, Vec3 skinOffset=0.0f, float skinExpand=0.0f, Vec4 color=Vec4(0.0f))
{
	if (rigid && g_rigidIndices.empty())
			g_rigidOffsets.push_back(0);

	Mesh* mesh = ImportMesh(file);

	if (!mesh)
	{
		printf("Could not open mesh for reading: %s\n", file);
		return;
	}

	int startIndex = int(g_positions.size());

	{
		mesh->Transform(RotationMatrix(rotation, Vec3(0.0f, 1.0f, 0.0f)));

		Vec3 meshLower, meshUpper;
		mesh->GetBounds(meshLower, meshUpper);

		Vec3 edges = meshUpper-meshLower;
		float maxEdge = max(max(edges.x, edges.y), edges.z);

		// put mesh at the origin and scale to specified size
		Matrix44 xform = ScaleMatrix(scale/maxEdge)*TranslationMatrix(Point3(-meshLower));

		mesh->Transform(xform);
		mesh->GetBounds(meshLower, meshUpper);

		// recompute expanded edges
		edges = meshUpper-meshLower;
		maxEdge = max(max(edges.x, edges.y), edges.z);

		// tweak spacing to avoid edge cases for particles laying on the boundary
		// just covers the case where an edge is a whole multiple of the spacing.
		float spacingEps = spacing*(1.0f - 1e-4f);

		// make sure to have at least one particle in each dimension
		int dx, dy, dz;
		dx = spacing > edges.x ? 1 : int(edges.x/spacingEps);
		dy = spacing > edges.y ? 1 : int(edges.y/spacingEps);
		dz = spacing > edges.z ? 1 : int(edges.z/spacingEps);

		int maxDim = max(max(dx, dy), dz);

		// expand border by two voxels to ensure adequate sampling at edges
		meshLower -= 2.0f*Vec3(spacing);
		meshUpper += 2.0f*Vec3(spacing);
		maxDim += 4;

		vector<uint32_t> voxels(maxDim*maxDim*maxDim);

		// we shift the voxelization bounds so that the voxel centers
		// lie symmetrically to the center of the object. this reduces the 
		// chance of missing features, and also better aligns the particles
		// with the mesh
		Vec3 meshOffset;
		meshOffset.x = 0.5f * (spacing - (edges.x - (dx-1)*spacing));
		meshOffset.y = 0.5f * (spacing - (edges.y - (dy-1)*spacing));
		meshOffset.z = 0.5f * (spacing - (edges.z - (dz-1)*spacing));
		meshLower -= meshOffset;

		//Voxelize(*mesh, dx, dy, dz, &voxels[0], meshLower - Vec3(spacing*0.05f) , meshLower + Vec3(maxDim*spacing) + Vec3(spacing*0.05f));
		Voxelize((const float*)&mesh->m_positions[0], mesh->m_positions.size(), (const int*)&mesh->m_indices[0], mesh->m_indices.size(), maxDim, maxDim, maxDim, &voxels[0], meshLower, meshLower + Vec3(maxDim*spacing));

		vector<float> sdf(maxDim*maxDim*maxDim);
		MakeSDF(&voxels[0], maxDim, maxDim, maxDim, &sdf[0]);

		for (int x=0; x < maxDim; ++x)
		{
			for (int y=0; y < maxDim; ++y)
			{
				for (int z=0; z < maxDim; ++z)
				{
					const int index = z*maxDim*maxDim + y*maxDim + x;

					// if voxel is marked as occupied the add a particle
					if (voxels[index])
					{
						if (rigid)
							g_rigidIndices.push_back(int(g_positions.size()));

						Vec3 position = lower + meshLower + spacing*Vec3(float(x) + 0.5f, float(y) + 0.5f, float(z) + 0.5f) + RandomUnitVector()*jitter;

						 // normalize the sdf value and transform to world scale
						Vec3 n = SafeNormalize(SampleSDFGrad(&sdf[0], maxDim, x, y, z));
						float d = sdf[index]*maxEdge;

						if (rigid)
							g_rigidLocalNormals.push_back(Vec4(n, d));

						g_positions.push_back(Vec4(position.x, position.y, position.z, invMass));						
						g_velocities.push_back(velocity);
						g_phases.push_back(phase);
					}
				}
			}
		}
		mesh->Transform(ScaleMatrix(1.0f + skinExpand)*TranslationMatrix(Point3(-0.5f*(meshUpper+meshLower))));
		mesh->Transform(TranslationMatrix(Point3(lower + 0.5f*(meshUpper+meshLower))));	
	}
	
	if (skin)
	{
		g_rigidMeshSize.push_back(mesh->GetNumVertices());

		int startVertex = 0;

		// append to mesh
		if (g_mesh)
		{
			startVertex = g_mesh->GetNumVertices();

			g_mesh->Transform(TranslationMatrix(Point3(skinOffset)));
			g_mesh->AddMesh(*mesh);

			delete mesh;
		}
		else
			g_mesh = mesh;

		mesh = g_mesh;

		const Colour colors[7] = 
		{
			Colour(0.0f, 0.5f, 1.0f),
			Colour(0.797f, 0.354f, 0.000f),			
			Colour(0.000f, 0.349f, 0.173f),
			Colour(0.875f, 0.782f, 0.051f),
			Colour(0.01f, 0.170f, 0.453f),
			Colour(0.673f, 0.111f, 0.000f),
			Colour(0.612f, 0.194f, 0.394f) 
		};

		for (uint32_t i=startVertex; i < mesh->GetNumVertices(); ++i)
		{
			int indices[g_numSkinWeights] = { -1, -1, -1, -1 };
			float distances[g_numSkinWeights] = {FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX };
			
			if (LengthSq(color) == 0.0f)
				mesh->m_colours[i] = 1.25f*colors[phase%7];
			else
				mesh->m_colours[i] = Colour(color);

			// find closest n particles
			for (size_t j=startIndex; j < g_positions.size(); ++j)
			{
				float dSq = LengthSq(Vec3(mesh->m_positions[i])-Vec3(g_positions[j]));

				// insertion sort
				int w=0;
				for (; w < 4; ++w)
					if (dSq < distances[w])
						break;
				
				if (w < 4)
				{
					// shuffle down
					for (int s=3; s > w; --s)
					{
						indices[s] = indices[s-1];
						distances[s] = distances[s-1];
					}

					distances[w] = dSq;
					indices[w] = int(j);				
				}
			}

			// weight particles according to distance
			float wSum = 0.0f;

			for (int w=0; w < 4; ++w)
			{				
				// convert to inverse distance
				distances[w] = 1.0f/(0.1f + powf(distances[w], .125f));

				wSum += distances[w];

			}

			float weights[4];
			for (int w=0; w < 4; ++w)
				weights[w] = distances[w]/wSum;

			for (int j=0; j < 4; ++j)
			{
				g_meshSkinIndices.push_back(indices[j]);
				g_meshSkinWeights.push_back(weights[j]);
			}
		}
	}
	else
	{
		delete mesh;
	}

	if (rigid)
	{
		g_rigidCoefficients.push_back(rigidStiffness);
		g_rigidOffsets.push_back(int(g_rigidIndices.size()));
	}
}

void SkinMesh()
{
	if (g_mesh)
	{
		int startVertex = 0;

		for (size_t r=0; r < g_rigidRotations.size(); ++r)
		{
			const Matrix33 rotation = g_rigidRotations[r];
			const int numVertices = g_rigidMeshSize[r];

			for (int i=startVertex; i < numVertices+startVertex; ++i)
			{
				Vec3 skinPos;

				for (int w=0; w < 4; ++w)
				{
					// small shapes can have < 4 particles
					if (g_meshSkinIndices[i*4+w] > -1)
					{
						assert(g_meshSkinWeights[i*4+w] < FLT_MAX);

						int index = g_meshSkinIndices[i*4+w];
						float weight = g_meshSkinWeights[i*4+w];

						skinPos += (rotation*(g_meshRestPositions[i]-Point3(g_restPositions[index])) + Vec3(g_positions[index]))*weight;
					}
				}

				g_mesh->m_positions[i] = Point3(skinPos);
			}

			startVertex += numVertices;
		}

		g_mesh->CalculateNormals();
	}
}

void CreateConvex(Vec3 halfEdge = Vec3(2.0f), Vec3 center=Vec3(0.0f), Vec4 quat=Vec4(0.0f, 0.0f, 0.0f, 1.0f), int flags=0)
{

	//cout << halfEdge.x << ' '<< halfEdge.y << ' ' << halfEdge.z << endl;
	//cout << center.x << ' '<< center.y << ' ' << center.z << endl;

	if (LengthSq(center) == 0.0f)
	{
		Vec3 lower, upper;
		GetParticleBounds(lower, upper);

		center = (lower+upper)*0.5f;
		center.y = 0.0f;
	}

	// create a box
	for (int i=0; i < 1; ++i)
	{
		int startPlane = int(g_convexPlanes.size());

		g_convexPlanes.push_back(Vec4(1.0f, 0.0f, 0.0f, -halfEdge.x));
		g_convexPlanes.push_back(Vec4(-1.0f, 0.0f, 0.0f, -halfEdge.x));

		g_convexPlanes.push_back(Vec4(0.0f, 1.0f, 0.0f, -halfEdge.y*0.5f));
		g_convexPlanes.push_back(Vec4(0.0f, -1.0f, 0.0f, -halfEdge.y));

		g_convexPlanes.push_back(Vec4(0.0f, 0.0f, 1.0f, -halfEdge.z));
		g_convexPlanes.push_back(Vec4(0.0f, 0.0f, -1.0f, -halfEdge.z));
		
		g_convexStarts.push_back(startPlane);
		g_convexLengths.push_back(6);

		g_convexPositions.push_back(Vec4(center.x, center.y, center.z, 0.0f));
		g_convexRotations.push_back(quat);

		g_convexPrevPositions.push_back(g_convexPositions.back());
		g_convexPrevRotations.push_back(g_convexRotations.back());

		// set aabbs
		ConvexMeshBuilder builder(&g_convexPlanes[startPlane]);
		builder(6);
			
		Vec3 lower(FLT_MAX), upper(-FLT_MAX);
		for (size_t v=0; v < builder.mVertices.size(); ++v)
		{
			Vec3 p =  rotate(Vec3(g_convexRotations.back()), g_convexRotations.back().w, builder.mVertices[v]) + Vec3(g_convexPositions.back());
			lower = Min(lower, p);
			upper = Max(upper, p);
		}
		g_convexAabbMin.push_back(Vec4(lower.x, lower.y, lower.z, 0.0f));
		g_convexAabbMax.push_back(Vec4(upper.x, upper.y, upper.z, 0.0f));		
		g_convexFlags.push_back(flags);
	}
}

void CreateSDF(const Mesh* mesh, uint32_t dim, Vec3 lower, Vec3 upper, float* sdf)
{
	if (mesh)
	{
		printf("Begin mesh voxelization\n");

		double startVoxelize = GetSeconds();

		uint32_t* volume = new uint32_t[dim*dim*dim];
		Voxelize((const float*)&mesh->m_positions[0], mesh->m_positions.size(), (const int*)&mesh->m_indices[0], mesh->m_indices.size(), dim, dim, dim, volume, lower, upper);

		printf("End mesh voxelization (%.2fs)\n", (GetSeconds()-startVoxelize));
	
		printf("Begin SDF gen (fast marching method)\n");

		double startSDF = GetSeconds();

		MakeSDF(volume, dim, dim, dim, sdf);

		printf("End SDF gen (%.2fs)\n", (GetSeconds()-startSDF));
	
		delete[] volume;
	}
}


void CreateRandomConvex(int numPlanes, Vec3 position, float minDist, float maxDist, Vec3 axis, float angle)
{
	// 12-kdop
	const Vec3 directions[] = { 
		Vec3(1.0f, 0.0f, 0.0f),
		Vec3(0.0f, 1.0f, 0.0f),
		Vec3(0.0f, 0.0f, 1.0f),
		Vec3(-1.0f, 0.0f, 0.0f),
		Vec3(0.0f, -1.0f, 0.0f),
		Vec3(0.0f, 0.0f, -1.0f),
		Vec3(1.0f, 1.0f, 0.0f),
		Vec3(-1.0f, -1.0f, 0.0f),
		Vec3(1.0f, 0.0f, 1.0f),
		Vec3(-1.0f, 0.0f, -1.0f),
		Vec3(0.0f, 1.0f, 1.0f),
		Vec3(0.0f, -1.0f, -1.0f),
	 };

	numPlanes = max(4, numPlanes);

	int index = int(g_convexLengths.size());
	int startPlane = int(g_convexPlanes.size());

	// create a box
	for (int i=0; i < numPlanes; ++i)
	{
		// pick random dir and distance
		Vec3 dir = Normalize(directions[i]);//RandomUnitVector();
		float dist = Randf(minDist, maxDist);

		g_convexPlanes.push_back(Vec4(dir.x, dir.y, dir.z, -dist));
	}

	g_convexStarts.push_back(startPlane);
	g_convexLengths.push_back(numPlanes);

	g_convexPositions.push_back(Vec4(position.x, position.y, position.z, 0.0f));
	g_convexRotations.push_back(QuatFromAxisAngle(axis, angle));

	g_convexPrevPositions.push_back(g_convexPositions.back());
	g_convexPrevRotations.push_back(g_convexRotations.back());

	// set aabbs
	ConvexMeshBuilder builder(&g_convexPlanes[startPlane]);
	builder(numPlanes);
			
	Vec3 lower(FLT_MAX), upper(-FLT_MAX);
	for (size_t v=0; v < builder.mVertices.size(); ++v)
	{
		Vec3 p =  rotate(Vec3(g_convexRotations[index]), g_convexRotations[index].w, builder.mVertices[v]) + Vec3(g_convexPositions[index]);
		lower = Min(lower, p);
		upper = Max(upper, p);
	}
	g_convexAabbMin.push_back(Vec4(lower.x, lower.y, lower.z, 0.0f));
	g_convexAabbMax.push_back(Vec4(upper.x, upper.y, upper.z, 0.0f));		
	g_convexFlags.push_back(0);
}

void CreateRandomBody(int numPlanes, Vec3 position, float minDist, float maxDist, Vec3 axis, float angle, float invMass, int phase, float stiffness)
{
	// 12-kdop
	const Vec3 directions[] = { 
		Vec3(1.0f, 0.0f, 0.0f),
		Vec3(0.0f, 1.0f, 0.0f),
		Vec3(0.0f, 0.0f, 1.0f),
		Vec3(-1.0f, 0.0f, 0.0f),
		Vec3(0.0f, -1.0f, 0.0f),
		Vec3(0.0f, 0.0f, -1.0f),
		Vec3(1.0f, 1.0f, 0.0f),
		Vec3(-1.0f, -1.0f, 0.0f),
		Vec3(1.0f, 0.0f, 1.0f),
		Vec3(-1.0f, 0.0f, -1.0f),
		Vec3(0.0f, 1.0f, 1.0f),
		Vec3(0.0f, -1.0f, -1.0f),
	 };

	numPlanes = max(4, numPlanes);

	vector<Vec4> planes;

	// create a box
	for (int i=0; i < numPlanes; ++i)
	{
		// pick random dir and distance
		Vec3 dir = Normalize(directions[i]);//RandomUnitVector();
		float dist = Randf(minDist, maxDist);

		planes.push_back(Vec4(dir.x, dir.y, dir.z, -dist));
	}

	// set aabbs
	ConvexMeshBuilder builder(&planes[0]);
	builder(numPlanes);
			
	int startIndex = int(g_positions.size());

	for (size_t v=0; v < builder.mVertices.size(); ++v)
	{
		Vec4 q = QuatFromAxisAngle(axis, angle);
		Vec3 p =  rotate(Vec3(q), q.w, builder.mVertices[v]) + position;

		g_positions.push_back(Vec4(p.x, p.y, p.z, invMass));
		g_velocities.push_back(0.0f);
		g_phases.push_back(phase);

		// add spring to all verts with higher index
		for (size_t i=v+1; i < builder.mVertices.size(); ++i)
		{
			int a = startIndex + int(v);
			int b = startIndex + int(i);

			g_springIndices.push_back(a);
			g_springIndices.push_back(b);
			g_springLengths.push_back(Length(builder.mVertices[v]-builder.mVertices[i]));
			g_springStiffness.push_back(stiffness);

		}
	}	

	for (size_t t=0; t < builder.mIndices.size(); ++t)
		g_triangles.push_back(startIndex + builder.mIndices[t]);		

	// lazy
	g_triangleNormals.resize(g_triangleNormals.size() + builder.mIndices.size()/3, Vec3(0.0f));
}

void CreateSDF(const char* meshFile, float scale, Vec3 lower, float expand=0.0f)
{
	// voxelize mesh
	Mesh* mesh = ImportMesh(meshFile);
	
	if (!mesh)
	{
		printf("Could not open mesh for reading: %s\n", meshFile);
	    return;
	}	
	else		
	{
		Vec3 minExtents, maxExtents, edges;
		mesh->GetBounds(minExtents, maxExtents);
		edges = maxExtents-minExtents;

		// normalize mesh scale
		float longestAxis = max(max(edges.x, edges.y), edges.z);
			
		mesh->Transform(TranslationMatrix(Point3(-Vec3(minExtents))));
		//mesh->Transform(RotationMatrix(-kPi*0.0f, Vec3(1.0f, 0.0f, 0.0f)));
		mesh->Transform(ScaleMatrix(scale/longestAxis));
		mesh->Transform(TranslationMatrix(Point3(lower)));

		mesh->GetBounds(minExtents, maxExtents);
		mesh->m_colours.resize(0);

		// store mesh 
		g_mesh = mesh;

		// square extents
		edges = maxExtents-minExtents;
		longestAxis = max(max(edges.x, edges.y), edges.z);
		edges = longestAxis;

		minExtents = minExtents - edges*0.1f;
		maxExtents = minExtents + edges*1.1f;
		edges = maxExtents-minExtents;

		// try and load the sdf from disc if it exists
		string sdfFile = string(meshFile, strrchr(meshFile, '.')) + ".pfm";
			
		PfmImage sdf;
		if (!PfmLoad(sdfFile.c_str(), sdf))
		{
			const int dim = 128;

			sdf.m_width = dim;
			sdf.m_height = dim;
			sdf.m_depth = dim;
			sdf.m_data = new float[dim*dim*dim];

			printf("Cooking SDF: %s - dim: %d^3\n", sdfFile.c_str(), dim);

			CreateSDF(mesh, dim, minExtents, maxExtents, sdf.m_data);

			PfmSave(sdfFile.c_str(), sdf);
		}

		printf("Loaded SDF, %d\n", sdf.m_width);

		assert(sdf.m_width == sdf.m_height && sdf.m_width == sdf.m_depth);

		// cheap collision offset
		int numVoxels = int(sdf.m_width*sdf.m_height*sdf.m_depth);
		for (int i=0; i < numVoxels; ++i)
			sdf.m_data[i] += expand;

		// set up flex collision shape
		g_shape.mWidth = sdf.m_width;
		g_shape.mHeight = sdf.m_height;
		g_shape.mDepth = sdf.m_depth;
		(Vec3&)g_shape.mLower = minExtents;
		(Vec3&)g_shape.mUpper = maxExtents;
		(Vec3&)g_shape.mInvEdgeLength = Vec3(1.0f/edges.x, 1.0f/edges.y, 1.0f/edges.z);
		g_shape.mField = sdf.m_data;
	}
}


/*added by eleanor*/
void CreateSDF2(const char* meshFile, float scale, Vec3 lower, float expand = 0.0f)
{
	// voxelize mesh
	Mesh* mesh = ImportMesh(meshFile);

	if (!mesh)
	{
		printf("Could not open mesh for reading: %s\n", meshFile);
		return;
	}
	else
	{
		Vec3 minExtents, maxExtents, edges;
		mesh->GetBounds(minExtents, maxExtents);
		edges = maxExtents - minExtents;

		// normalize mesh scale
		float longestAxis = max(max(edges.x, edges.y), edges.z);

		mesh->Transform(TranslationMatrix(Point3(-Vec3(minExtents))));
		//mesh->Transform(RotationMatrix(-kPi*0.0f, Vec3(1.0f, 0.0f, 0.0f)));
		mesh->Transform(ScaleMatrix(scale / longestAxis));
		mesh->Transform(TranslationMatrix(Point3(lower)));

		mesh->GetBounds(minExtents, maxExtents);
		mesh->m_colours.resize(0);

		/*add begin*/
		g_numPoints = mesh->GetNumVertices();
		g_numTriangles = mesh->GetNumFaces();

		g_saturations.resize(g_numTriangles, 0.0);
		g_triangleNeighbours.resize(g_numTriangles, Vec3(-1.0, -1.0, -1.0));

		g_pointTriangleNums = mesh->m_pointTriangleNums;
		g_pointTriangles = mesh->m_pointTriangles;
		g_trianglePoints = mesh->m_trianglePoints;

		mesh->m_pointTriangleNums.resize(0);
		mesh->m_pointTriangles.resize(0);
		mesh->m_trianglePoints.resize(0);

		/*add end*/

		// store mesh 
		g_mesh = mesh;

		// square extents
		edges = maxExtents - minExtents;
		longestAxis = max(max(edges.x, edges.y), edges.z);
		edges = longestAxis;

		minExtents = minExtents - edges*0.1f;
		maxExtents = minExtents + edges*1.1f;
		edges = maxExtents - minExtents;

		// try and load the sdf from disc if it exists
		string sdfFile = string(meshFile, strrchr(meshFile, '.')) + ".pfm";

		PfmImage sdf;
		if (!PfmLoad(sdfFile.c_str(), sdf))
		{
			const int dim = 128;

			sdf.m_width = dim;
			sdf.m_height = dim;
			sdf.m_depth = dim;
			sdf.m_data = new float[dim*dim*dim];

			printf("Cooking SDF: %s - dim: %d^3\n", sdfFile.c_str(), dim);

			CreateSDF(mesh, dim, minExtents, maxExtents, sdf.m_data);

			PfmSave(sdfFile.c_str(), sdf);
		}

		printf("Loaded SDF, %d\n", sdf.m_width);

		assert(sdf.m_width == sdf.m_height && sdf.m_width == sdf.m_depth);

		// cheap collision offset
		int numVoxels = int(sdf.m_width*sdf.m_height*sdf.m_depth);
		for (int i = 0; i < numVoxels; ++i)
			sdf.m_data[i] += expand;

		// set up flex collision shape
		g_shape.mWidth = sdf.m_width;
		g_shape.mHeight = sdf.m_height;
		g_shape.mDepth = sdf.m_depth;
		(Vec3&)g_shape.mLower = minExtents;
		(Vec3&)g_shape.mUpper = maxExtents;
		(Vec3&)g_shape.mInvEdgeLength = Vec3(1.0f / edges.x, 1.0f / edges.y, 1.0f / edges.z);
		g_shape.mField = sdf.m_data;
	}
}
/*add end*/

inline int GridIndex(int x, int y, int dx) { return y*dx + x; }

void CreateSpring(int i, int j, float stiffness, float give=0.0f)
{
	g_springIndices.push_back(i);
	g_springIndices.push_back(j);
	g_springLengths.push_back((1.0f+give)*Length(Vec3(g_positions[i])-Vec3(g_positions[j])));
	g_springStiffness.push_back(stiffness);	
}

void CreateSpringGrid(Vec3 lower, int dx, int dy, int dz, float radius, int phase, float stretchStiffness, float bendStiffness, float shearStiffness, Vec3 velocity, float invMass)
{
	int baseIndex = int(g_positions.size());

	for (int z=0; z < dz; ++z)
	{
		for (int y=0; y < dy; ++y)
		{
			for (int x=0; x < dx; ++x)
			{
				Vec3 position = lower + radius*Vec3(float(x), float(z), float(y));

				g_positions.push_back(Vec4(position.x, position.y, position.z, invMass));
				g_velocities.push_back(velocity);
				g_phases.push_back(phase);
				
				/*added by eleanor*/

				g_uvs.push_back(Vec3((float)x / dx * 8, (float)y / dy * 8, 0.0));
				//g_uvs.push_back(Vec3((float)x / dx * 4, (float)y / dy * 4, 0.0));
				//g_uvs.push_back(Vec3((float)x / dx, (float)y / dy, 0.0));

				/*add end*/

				if (x > 0 && y > 0)
				{
					g_triangles.push_back(baseIndex + GridIndex(x-1, y-1, dx));
					g_triangles.push_back(baseIndex + GridIndex(x, y-1, dx));
					g_triangles.push_back(baseIndex + GridIndex(x, y, dx));
					
					g_triangles.push_back(baseIndex + GridIndex(x-1, y-1, dx));
					g_triangles.push_back(baseIndex + GridIndex(x, y, dx));
					g_triangles.push_back(baseIndex + GridIndex(x-1, y, dx));

					g_triangleNormals.push_back(Vec3(0.0f, 1.0f, 0.0f));
					g_triangleNormals.push_back(Vec3(0.0f, 1.0f, 0.0f));
				}
			}
		}
	}	

	// horizontal
	for (int y=0; y < dy; ++y)
	{
		for (int x=0; x < dx; ++x)
		{
			int index0 = y*dx + x;

			if (x > 0)
			{
				int index1 = y*dx + x - 1;
				CreateSpring(baseIndex + index0, baseIndex + index1, stretchStiffness);
			}

			if (x > 1)
			{
				int index2 = y*dx + x - 2;
				CreateSpring(baseIndex + index0, baseIndex + index2, bendStiffness);
			}

			if (y > 0 && x < dx-1)
			{
				int indexDiag = (y-1)*dx + x + 1;
				CreateSpring(baseIndex + index0, baseIndex + indexDiag, shearStiffness);
			}

			if (y > 0 && x > 0)
			{
				int indexDiag = (y-1)*dx + x - 1;
				CreateSpring(baseIndex + index0, baseIndex + indexDiag, shearStiffness);
			}
		}
	}

	// vertical
	for (int x=0; x < dx; ++x)
	{
		for (int y=0; y < dy; ++y)
		{
			int index0 = y*dx + x;

			if (y > 0)
			{
				int index1 = (y-1)*dx + x;
				CreateSpring(baseIndex + index0, baseIndex + index1, stretchStiffness);
			}

			if (y > 1)
			{
				int index2 = (y-2)*dx + x;
				CreateSpring(baseIndex + index0, baseIndex + index2, bendStiffness);
			}
		}
	}	
}

/*added by eleanor*/
inline int GridIndex(int x, int y, int z, int dx, int dy) { return z * dx * dy + y * dx + x; }


void renewTrianglePoints(int idxp, int idxpt, int idxt) {
	Vec4 tmp = g_pointTriangles[idxp];
	switch (idxpt)
	{
	case 0:
		tmp.w = idxt;
		break;
	case 1:
		tmp.x = idxt;
		break;
	case 2:
		tmp.y = idxt;
		break;
	case 3:
		tmp.z = idxt;
		break;
	default:
		break;
	}
	g_pointTriangles[idxp] = tmp;
}
void AddTriangleToPoints(int idxp, int idxt1, int idxt2) {
	int idx = g_pointTriangleNums[idxp];
	if (idxt1 > -1) {
		if (idx < 4) {
			renewTrianglePoints(idxp * 2, idx, idxt1);
			g_pointTriangleNums[idxp] = idx + 1;
			idx++;
		}
		else {
			renewTrianglePoints(idxp * 2 + 1, idx - 4, idxt1);
			g_pointTriangleNums[idxp] = idx + 1;
			idx++;
		}
	}
	if (idxt2 > -1) {
		if (idx < 4) {
			renewTrianglePoints(idxp * 2, idx, idxt2);
			g_pointTriangleNums[idxp] = idx + 1;
			idx++;
		}
		else {
			renewTrianglePoints(idxp * 2 + 1, idx - 4, idxt2);
			g_pointTriangleNums[idxp] = idx + 1;
			idx++;
		}
	}
}
//add relations of triangles & points
void CreateSpringGrid2(Vec3 lower, int dx, int dy, int dz, float radius, int phase, float stretchStiffness, float bendStiffness, float shearStiffness, Vec3 velocity, float invMass) {

	int baseIndex = int(g_positions.size());

	int index = 0;

	for (int z = 0; z < dz; ++z)
	{
		for (int y = 0; y < dy; ++y)
		{
			for (int x = 0; x < dx; ++x)
			{
				Vec3 position = lower + radius*Vec3(float(x), float(z), float(y));

				g_positions.push_back(Vec4(position.x, position.y, position.z, invMass));
				g_velocities.push_back(velocity);
				g_phases.push_back(phase);

				/*add begin*/

				//g_uvs.push_back(Vec3((float)x / (dx - 1) * 16, (float)y / (dy - 1) * 16, 0.0));
				//g_uvs.push_back(Vec3((float)x / (dx - 1) * 8, (float)y / (dy - 1) * 8, 0.0));
				//g_uvs.push_back(Vec3((float)x / (dx - 1) * 4, (float)y / (dy - 1) * 4, 0.0));
				//g_uvs.push_back(Vec3((float)x / (dx - 1), (float)y / (dy - 1), 0.0));

				//g_uvs.push_back(Vec3((float)x / dx * 16, (float)(dy - y) / dy * 16, 0.0));
				//g_uvs.push_back(Vec3((float)x / dx * 8, (float)(dy - y) / dy * 8, 0.0));
				//g_uvs.push_back(Vec3((float)x / dx * 4, (float)(dy - y) / dy * 4, 0.0));
				g_uvs.push_back(Vec3((float)x / dx, (float)(dy - y) / dy, 0.0));

				int i = GridIndex(x, y, dx);
				g_pointTriangleNums[i] = 0;
				g_pointTriangles[i * 2] = Vec4(-1.0, -1.0, -1.0, -1.0);
				g_pointTriangles[i * 2 + 1] = Vec4(-1.0, -1.0, -1.0, -1.0);


				/*add end*/

				if (x > 0 && y > 0)
				{
					/*add begin*/
					
					int p1 = GridIndex(x - 1, y - 1, dx);
					int p2 = GridIndex(x, y - 1, dx);
					int p3 = GridIndex(x - 1, y, dx);
					int p4 = GridIndex(x, y, dx);

					/*add end*/

					g_triangles.push_back(baseIndex + GridIndex(x - 1, y - 1, dx));
					g_triangles.push_back(baseIndex + GridIndex(x, y - 1, dx));
					g_triangles.push_back(baseIndex + GridIndex(x, y, dx));

					g_triangles.push_back(baseIndex + GridIndex(x - 1, y - 1, dx));
					g_triangles.push_back(baseIndex + GridIndex(x, y, dx));
					g_triangles.push_back(baseIndex + GridIndex(x - 1, y, dx));

					g_triangleNormals.push_back(Vec3(0.0f, 1.0f, 0.0f));
					g_triangleNormals.push_back(Vec3(0.0f, 1.0f, 0.0f));

					/*add begin*/

					g_saturations.push_back(0.0);
					g_saturations.push_back(0.0);

					g_triangleNeighbours.push_back(Vec3(-1.0, -1.0, -1.0));
					g_triangleNeighbours.push_back(Vec3(-1.0, -1.0, -1.0));

					g_trianglePoints.push_back(Vec3(p1, p2, p4));
					g_trianglePoints.push_back(Vec3(p1, p3, p4));

					AddTriangleToPoints(p1, index, index + 1);
					AddTriangleToPoints(p2, index, -1);
					AddTriangleToPoints(p3, index + 1, -1);
					AddTriangleToPoints(p4, index, index + 1);
					index += 2;

					/*add end*/
				}
			}
		}
	}

	// horizontal
	for (int y = 0; y < dy; ++y)
	{
		for (int x = 0; x < dx; ++x)
		{
			int index0 = y*dx + x;

			if (x > 0)
			{
				int index1 = y*dx + x - 1;
				CreateSpring(baseIndex + index0, baseIndex + index1, stretchStiffness);
			}

			if (x > 1)
			{
				int index2 = y*dx + x - 2;
				CreateSpring(baseIndex + index0, baseIndex + index2, bendStiffness);
			}

			if (y > 0 && x < dx - 1)
			{
				int indexDiag = (y - 1)*dx + x + 1;
				CreateSpring(baseIndex + index0, baseIndex + indexDiag, shearStiffness);
			}

			if (y > 0 && x > 0)
			{
				int indexDiag = (y - 1)*dx + x - 1;
				CreateSpring(baseIndex + index0, baseIndex + indexDiag, shearStiffness);
			}
		}
	}

	// vertical
	for (int x = 0; x < dx; ++x)
	{
		for (int y = 0; y < dy; ++y)
		{
			int index0 = y*dx + x;

			if (y > 0)
			{
				int index1 = (y - 1)*dx + x;
				CreateSpring(baseIndex + index0, baseIndex + index1, stretchStiffness);
			}

			if (y > 1)
			{
				int index2 = (y - 2)*dx + x;
				CreateSpring(baseIndex + index0, baseIndex + index2, bendStiffness);
			}
		}
	}

}

//add relations of triangles & points
//verticle
void CreateSpringGrid3(Vec3 lower, int dx, int dy, int dz, float radius, int phase, float stretchStiffness, float bendStiffness, float shearStiffness, Vec3 velocity, float invMass) {

	int baseIndex = int(g_positions.size());

	int index = 0;

	for (int z = 0; z < dz; ++z)
	{
		for (int y = 0; y < dy; ++y)
		{
			for (int x = 0; x < dx; ++x)
			{
				Vec3 position = lower + radius*Vec3(float(x), float(-y), float(z));

				g_positions.push_back(Vec4(position.x, position.y, position.z, invMass));
				g_velocities.push_back(velocity);
				g_phases.push_back(phase);

				/*add begin*/

				//g_uvs.push_back(Vec3((float)x / (dx - 1) * 16, (float)y / (dy - 1) * 16, 0.0));
				//g_uvs.push_back(Vec3((float)x / (dx - 1) * 8, (float)y / (dy - 1) * 8, 0.0));
				//g_uvs.push_back(Vec3((float)x / (dx - 1) * 4, (float)y / (dy - 1) * 4, 0.0));
				//g_uvs.push_back(Vec3((float)x / (dx - 1), (float)y / (dy - 1), 0.0));

				//g_uvs.push_back(Vec3((float)x / dx * 16, (float)(dy - y) / dy * 16, 0.0));
				//g_uvs.push_back(Vec3((float)x / dx * 8, (float)(dy - y) / dy * 8, 0.0));
				//g_uvs.push_back(Vec3((float)x / dx * 4, (float)(dy - y) / dy * 4, 0.0));
				//g_uvs.push_back(Vec3((float)x / dx * 2, (float)(dy - y) / dy * 2, 0.0));
				g_uvs.push_back(Vec3((float)x / dx, (float)(dy - y) / dy, 0.0));

				int i = GridIndex(x, y, dx);
				g_pointTriangleNums[i] = 0;
				g_pointTriangles[i * 2] = Vec4(-1.0, -1.0, -1.0, -1.0);
				g_pointTriangles[i * 2 + 1] = Vec4(-1.0, -1.0, -1.0, -1.0);


				/*add end*/

				if (x > 0 && y > 0)
				{
					/*add begin*/

					int p1 = GridIndex(x - 1, y - 1, dx);
					int p2 = GridIndex(x, y - 1, dx);
					int p3 = GridIndex(x - 1, y, dx);
					int p4 = GridIndex(x, y, dx);

					/*add end*/

					g_triangles.push_back(baseIndex + GridIndex(x - 1, y - 1, dx));
					g_triangles.push_back(baseIndex + GridIndex(x, y - 1, dx));
					g_triangles.push_back(baseIndex + GridIndex(x, y, dx));

					g_triangles.push_back(baseIndex + GridIndex(x - 1, y - 1, dx));
					g_triangles.push_back(baseIndex + GridIndex(x, y, dx));
					g_triangles.push_back(baseIndex + GridIndex(x - 1, y, dx));

					g_triangleNormals.push_back(Vec3(0.0f, 1.0f, 0.0f));
					g_triangleNormals.push_back(Vec3(0.0f, 1.0f, 0.0f));

					/*add begin*/

					g_saturations.push_back(0.0);
					g_saturations.push_back(0.0);

					g_triangleNeighbours.push_back(Vec3(-1.0, -1.0, -1.0));
					g_triangleNeighbours.push_back(Vec3(-1.0, -1.0, -1.0));

					g_trianglePoints.push_back(Vec3(p1, p2, p4));
					g_trianglePoints.push_back(Vec3(p1, p3, p4));

					AddTriangleToPoints(p1, index, index + 1);
					AddTriangleToPoints(p2, index, -1);
					AddTriangleToPoints(p3, index + 1, -1);
					AddTriangleToPoints(p4, index, index + 1);
					index += 2;

					/*add end*/
				}
			}
		}
	}

	// horizontal
	for (int y = 0; y < dy; ++y)
	{
		for (int x = 0; x < dx; ++x)
		{
			int index0 = y*dx + x;

			if (x > 0)
			{
				int index1 = y*dx + x - 1;
				CreateSpring(baseIndex + index0, baseIndex + index1, stretchStiffness);
			}

			if (x > 1)
			{
				int index2 = y*dx + x - 2;
				CreateSpring(baseIndex + index0, baseIndex + index2, bendStiffness);
			}

			if (y > 0 && x < dx - 1)
			{
				int indexDiag = (y - 1)*dx + x + 1;
				CreateSpring(baseIndex + index0, baseIndex + indexDiag, shearStiffness);
			}

			if (y > 0 && x > 0)
			{
				int indexDiag = (y - 1)*dx + x - 1;
				CreateSpring(baseIndex + index0, baseIndex + indexDiag, shearStiffness);
			}
		}
	}

	// vertical
	for (int x = 0; x < dx; ++x)
	{
		for (int y = 0; y < dy; ++y)
		{
			int index0 = y*dx + x;

			if (y > 0)
			{
				int index1 = (y - 1)*dx + x;
				CreateSpring(baseIndex + index0, baseIndex + index1, stretchStiffness);
			}

			if (y > 1)
			{
				int index2 = (y - 2)*dx + x;
				CreateSpring(baseIndex + index0, baseIndex + index2, bendStiffness);
			}
		}
	}

}



/*add end*/


void CreateRope(Rope& rope, Vec3 start, Vec3 dir, float stiffness, int segments, float length, int phase, float spiralAngle=0.0f, float invmass=1.0f, float give=0.075f)
{
	rope.mIndices.push_back(int(g_positions.size()));

	g_positions.push_back(Vec4(start.x, start.y, start.z, invmass));
	g_velocities.push_back(0.0f);
	g_phases.push_back(phase);//int(g_positions.size()));
	
	Vec3 left, right;
	BasisFromVector(dir, &left, &right);

	float segmentLength = length/segments;
	Vec3 spiralAxis = dir;
	float spiralHeight = spiralAngle/(2.0f*kPi)*(length/segments);

	if (spiralAngle > 0.0f)
		dir = left;

	Vec3 p = start;

	for (int i=0; i < segments; ++i)
	{
		int prev = int(g_positions.size())-1;

		p += dir*segmentLength;

		// rotate 
		if (spiralAngle > 0.0f)
		{
			p += spiralAxis*spiralHeight;

			dir = RotationMatrix(spiralAngle, spiralAxis)*dir;
		}

		rope.mIndices.push_back(int(g_positions.size()));

		g_positions.push_back(Vec4(p.x, p.y, p.z, 1.0f));
		g_velocities.push_back(0.0f);
		g_phases.push_back(phase);//int(g_positions.size()));

		// stretch
		CreateSpring(prev, prev+1, stiffness, give);

		// tether
		//if (i > 0 && i%4 == 0)
			//CreateSpring(prev-3, prev+1, -0.25f);
		
		// bending spring
		if (i > 0)
			CreateSpring(prev-1, prev+1, stiffness*0.5f, give);
	}
}

namespace
{
	struct Tri
	{
		int a;
		int b;
		int c;

		Tri(int a, int b, int c) : a(a), b(b), c(c) {}

		bool operator < (const Tri& rhs)
		{
			if (a != rhs.a)
				return a < rhs.a;
			else if (b != rhs.b)
				return b < rhs.b;
			else
				return c < rhs.c;
		}
	};
}


namespace
{
	struct TriKey
	{
		int orig[3];
		int indices[3];

		TriKey(int a, int b, int c)		
		{
			orig[0] = a;
			orig[1] = b;
			orig[2] = c;

			indices[0] = a;
			indices[1] = b;
			indices[2] = c;

			std::sort(indices, indices+3);
		}			

		bool operator < (const TriKey& rhs) const
		{
			if (indices[0] != rhs.indices[0])
				return indices[0] < rhs.indices[0];
			else if (indices[1] != rhs.indices[1])
				return indices[1] < rhs.indices[1];
			else
				return indices[2] < rhs.indices[2];
		}
	};
}

void CreateTetMesh(const char* filename, Vec3 lower, float scale, float stiffness, int phase)
{
	FILE* f = fopen(filename, "r");

	char line[2048];

	if (f)
	{
		typedef std::map<TriKey, int> TriMap;
		TriMap triCount;

		const int vertOffset = g_positions.size();

		Vec3 meshLower(FLT_MAX);
		Vec3 meshUpper(-FLT_MAX);

		bool firstTet = true;

		while (!feof(f))
		{
			if (fgets(line, 2048, f))
			{
				switch(line[0])
				{
				case '#':
					break;
				case 'v':
					{
						Vec3 pos;
						sscanf(line, "v %f %f %f", &pos.x, &pos.y, &pos.z);

						g_positions.push_back(Vec4(pos.x, pos.y, pos.z, 1.0f));
						g_velocities.push_back(0.0f);
						g_phases.push_back(phase);

						meshLower = Min(pos, meshLower);
						meshUpper = Max(pos, meshUpper);
						break;
					}
				case 't':
					{
						if (firstTet)
						{
							Vec3 edges = meshUpper-meshLower;
							float maxEdge = max(edges.x, max(edges.y, edges.z));

							// normalize positions
							for (int i=vertOffset; i < int(g_positions.size()); ++i)
							{
								Vec3 p = lower + (Vec3(g_positions[i])-meshLower)*scale/maxEdge;
								g_positions[i] = Vec4(p, g_positions[i].w);
							}

							firstTet = false;
						}

						int indices[4];
						sscanf(line, "t %d %d %d %d", &indices[0], &indices[1], &indices[2], &indices[3]);

						indices[0] += vertOffset;
						indices[1] += vertOffset;
						indices[2] += vertOffset;
						indices[3] += vertOffset;

						CreateSpring(indices[0], indices[1], stiffness);
						CreateSpring(indices[0], indices[2], stiffness);
						CreateSpring(indices[0], indices[3], stiffness);
				
						CreateSpring(indices[1], indices[2], stiffness);
						CreateSpring(indices[1], indices[3], stiffness);
						CreateSpring(indices[2], indices[3], stiffness);

						TriKey k1(indices[0], indices[2], indices[1]);
						triCount[k1] += 1;

						TriKey k2(indices[1], indices[2], indices[3]);
						triCount[k2] += 1;

						TriKey k3(indices[0], indices[1], indices[3]);
						triCount[k3] += 1;

						TriKey k4(indices[0], indices[3], indices[2]);
						triCount[k4] += 1;

						break;
					}
				}
			}
		}

		for (TriMap::iterator iter=triCount.begin(); iter != triCount.end(); ++iter)
		{
			TriKey key = iter->first;

			// only output faces that are referenced by one tet (open faces)
			if (iter->second == 1)
			{
				g_triangles.push_back(key.orig[0]);
				g_triangles.push_back(key.orig[1]);
				g_triangles.push_back(key.orig[2]);
				g_triangleNormals.push_back(0.0f);
			}
		}


		fclose(f);
	}
}

// unproject a pixel coordinate using the current OpenGL viewing transforms
void GetViewRay(int x, int y, Vec3& origin, Vec3& dir)
{
	double modelview[16];
	glGetDoublev(GL_MODELVIEW_MATRIX, modelview);

	double projection[16];
	glGetDoublev(GL_PROJECTION_MATRIX, projection);

	int viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport);

	double nearPos[3];
	gluUnProject(double(x), double(y), 0.0f, modelview, projection, viewport, &nearPos[0], &nearPos[1], &nearPos[2]);

	double farPos[3];
	gluUnProject(double(x), double(y), 1.0f, modelview, projection, viewport, &farPos[0], &farPos[1], &farPos[2]);
	
	origin = Vec3(float(nearPos[0]), float(nearPos[1]), float(nearPos[2]));
	dir = Normalize(Vec3(float(farPos[0]-nearPos[0]), float(farPos[1]-nearPos[1]), float(farPos[2]-nearPos[2])));
}

// finds the closest particle to a view ray
int PickParticle(Vec3 origin, Vec3 dir, Vec4* particles, int* phases, int n, float radius, float &t)
{
	float maxDistSq = radius*radius;
	float minT = FLT_MAX;
	int minIndex = -1;

	for (int i=0; i < n; ++i)
	{
		if (phases[i] & eFlexPhaseFluid)
			continue;

		Vec3 delta = Vec3(particles[i])-origin;
		float t = Dot(delta, dir);

		if (t > 0.0f)
		{
			Vec3 perp = delta - t*dir;

			float dSq = LengthSq(perp);

			if (dSq < maxDistSq && t < minT)
			{
				minT = t;
				minIndex = i;
			}
		}
	}

	t = minT;

	return minIndex;
}

// calculates local space positions given a set of particles and rigid indices
void CalculateRigidOffsets(const Vec4* restPositions, const int* offsets, const int* indices, int numRigids, Vec3* localPositions)
{
	int count = 0;

	for (int r=0; r < numRigids; ++r)
	{
		const int startIndex = offsets[r];
		const int endIndex = offsets[r+1];

		const int n = endIndex-startIndex;

		assert(n);

		Vec3 com;
	
		for (int i=startIndex; i < endIndex; ++i)
		{
			const int r = indices[i];

			com += Vec3(restPositions[r]);
		}

		com /= float(n);

		for (int i=startIndex; i < endIndex; ++i)
		{
			const int r = indices[i];

			localPositions[count++] = Vec3(restPositions[r])-com;
		}
	}
}

void DrawImguiString(int x, int y, Vec3 color, int align, const char* s, ...)
{
	char buf[2048];

	va_list args;

	va_start(args, s);
	vsnprintf(buf, 2048, s, args);
	va_end(args);

	glColor3fv(color);
	//DrawStringS(x ,y, buf);
	imguiDrawText(x, y, align, buf, imguiRGBA((unsigned char)(color.x*255), (unsigned char)(color.y*255), (unsigned char)(color.z*255)));
}


/*added by eleanor*/

//initiate
void printVec3(Vec3 x) {
	cout << x.x << ' ' << x.y << ' ' << x.z << endl;
}

int trans(int ip, int idx) {
	Vec4 tmp1 = g_pointTriangles[ip * 2];
	Vec4 tmp2 = g_pointTriangles[ip * 2 + 1];
	switch (idx)
	{
	case 0:
		return int(tmp1.w);
	case 1:
		return int(tmp1.x);
	case 2:
		return int(tmp1.y);
	case 3:
		return int(tmp1.z);
	case 4:
		return int(tmp2.w);
	case 5:
		return int(tmp2.x);
	case 6:
		return int(tmp2.y);
	case 7:
		return int(tmp2.z);

	default:
		break;
	}
	return 0;
}
void renewNeighbour(int it, int p, int in) {
	Vec3 neighbours = g_triangleNeighbours[it];
	if (p == 0) {
		//adjacent edge is xy
		neighbours.x = in;
	}
	else if (p == 1) {
		//adjacent edge is yz
		neighbours.y = in;
	}
	else if (p == 2) {
		//adjacent edge is zx
		neighbours.z = in;
	}
	g_triangleNeighbours[it] = neighbours;
}
void checkNeighbour(int ip, int it1, int it2) {
	int idx1 = trans(ip, it1);
	int idx2 = trans(ip, it2);

	//cout << idx1 << ' ' << idx2 << endl;

	Vec3 point1 = g_trianglePoints[idx1];
	Vec3 point2 = g_trianglePoints[idx2];

	//printVec3(point1);
	//printVec3(point2);

	int p1 = -1, p2 = -1;

	if (point1.x == point2.x && point1.y == point2.y) {
		p1 = 0, p2 = 0;
	}
	else if (point1.x == point2.y && point1.y == point2.z) {
		p1 = 0; p2 = 1;
	}
	else if (point1.x == point2.x && point1.y == point2.z) {
		p1 = 0; p2 = 2;
	}
	else if (point1.y == point2.x && point1.z == point2.y) {
		p1 = 1; p2 = 0;
	}
	else if (point1.y == point2.y && point1.z == point2.z) {
		p1 = 1; p2 = 1;
	}
	else if (point1.y == point2.x && point1.z == point2.z) {
		p1 = 1; p2 = 2;
	}
	else if (point1.x == point2.x && point1.z == point2.y) {
		p1 = 2; p2 = 0;
	}
	else if (point1.x == point2.y && point1.z == point2.z) {
		p1 = 2; p2 = 1; 
	}
	else if (point1.x == point2.x && point1.z == point2.z) {
		p1 = 2; p2 = 2;
	}

	if (p1 >= 0) {
		renewNeighbour(idx1, p1, idx2);
	}
	if (p2 >= 0) {
		renewNeighbour(idx2, p2, idx1);
	}
}
void CalculateTriangleNeighbours() {
	if (g_triangleNeighbours.size() == 0) {
		g_triangleNeighbours.resize(g_numTriangles);
	}

	for (int i = 0; i < g_numPoints; i++) {
		int num = g_pointTriangleNums[i];
		for (int j = 0; j < num; j++) {
			for (int k = j + 1; k < num; k++) {
				checkNeighbour(i, j, k);
			}
		}
	}
}

//float getMin(float x, float y) {
//	if (x < y) return x;
//	else return y;
//}
//
//float getMax(float x, float y) {
//	if (x > y) return x;
//	else return y;
//}

//colors

void CalculateColors() {
	float maxSaturation = g_maxSaturation * g_kMaxAbsorption;

	Vec4 color = Vec4(g_clothColor, 1.0f);
	Vec4 colorBase = color / maxSaturation;

	for (int i = 0; i < g_numPoints; i++) {
		float saturation = 0.0;
		int num = g_pointTriangleNums[i];

		Vec4 tmp1 = g_pointTriangles[i * 2];
		Vec4 tmp2 = g_pointTriangles[i * 2 + 1];

		switch (num) {
		case 8:
			saturation += g_saturations[int(tmp2.z)];
		case 7:
			saturation += g_saturations[int(tmp2.y)];
		case 6:
			saturation += g_saturations[int(tmp2.x)];
		case 5:
			saturation += g_saturations[int(tmp2.w)];
		case 4:
			saturation += g_saturations[int(tmp1.z)];
		case 3:
			saturation += g_saturations[int(tmp1.y)];
		case 2:
			saturation += g_saturations[int(tmp1.x)];
		case 1:
			saturation += g_saturations[int(tmp1.w)];
		default:
			break;
		}

		if (num > 0) {
			saturation = saturation / num;
		}

		if (saturation < 0.0) saturation = 0.0;
		if (saturation > maxSaturation) saturation = maxSaturation;

		if (g_markColor && saturation > 0.0) {
			g_colors[i] = g_markColors[(int)(saturation / maxSaturation * 10)];
		}
		else {
			g_colors[i] = colorBase * (maxSaturation - saturation);
		}
	}
}
void CalculateMeshColors() {
	if (g_mesh->m_colours.size() == 0) {
		g_mesh->m_colours.resize(g_numPoints);
	}

	float maxSaturation = g_maxSaturation * g_kMaxAbsorption;

	Vec4 color = Vec4(g_clothColor, 1.0f);
	Vec4 colorBase = color / maxSaturation;

	for (int i = 0; i < g_numPoints; i++) {
		float saturation = 0.0;
		int num = g_pointTriangleNums[i];
		Vec4 tmp1 = g_pointTriangles[i * 2];
		Vec4 tmp2 = g_pointTriangles[i * 2 + 1];

		switch (num) {
		case 8:
			saturation += g_saturations[int(tmp2.z)];
		case 7:
			saturation += g_saturations[int(tmp2.y)];
		case 6:
			saturation += g_saturations[int(tmp2.x)];
		case 5:
			saturation += g_saturations[int(tmp2.w)];
		case 4:
			saturation += g_saturations[int(tmp1.z)];
		case 3:
			saturation += g_saturations[int(tmp1.y)];
		case 2:
			saturation += g_saturations[int(tmp1.x)];
		case 1:
			saturation += g_saturations[int(tmp1.w)];
		default:
			break;
		}

		if (saturation < 0.0) saturation = 0.0;
		if (saturation > maxSaturation) saturation = maxSaturation;

		if (g_markColor && saturation > 0.0) {
			Vec4 color = g_markColors[(int)(saturation / maxSaturation * 10)];
			g_mesh->m_colours[i] = Colour(color.x, color.y, color.z, color.w);
		}
		else {
			Vec4 color = colorBase * (maxSaturation - saturation);
			g_mesh->m_colours[i] = Colour(color.x, color.y, color.z, color.w);
		}
	}
}


//absorb
bool Collide(int i, int j) {
	Vec4 posX = g_positions[i];
	Vec4 posY = g_positions[j];

	if (posX.y - posY.y > g_params.mSolidRestDistance) return false;
	if (posX.x - posY.x > g_params.mSolidRestDistance) return false;
	if (posX.z - posY.z > g_params.mSolidRestDistance) return false;

	float dist = sqrt(sqr(posX.x - posY.x) + sqr(posX.y - posY.y) + sqr(posX.z - posY.z));

	if (dist <= g_params.mSolidRestDistance) {
		return true;
	}

	return false;

}
void UpdateSaturations(int idx) {
	int num = g_pointTriangleNums[idx];

	Vec4 tmp1 = g_pointTriangles[idx * 2];
	Vec4 tmp2 = g_pointTriangles[idx * 2 + 1];

	float tmp = g_mDrip / num;

	switch (num)
	{
	case 8:
		g_saturations[int(tmp2.z)] += tmp;
	case 7:
		g_saturations[int(tmp2.y)] += tmp;
	case 6:
		g_saturations[int(tmp2.x)] += tmp;
	case 5:
		g_saturations[int(tmp2.w)] += tmp;
	case 4:
		g_saturations[int(tmp1.z)] += tmp;
	case 3:
		g_saturations[int(tmp1.y)] += tmp;
	case 2:
		g_saturations[int(tmp1.x)] += tmp;
	case 1:
		g_saturations[int(tmp1.w)] += tmp;
	default:
		break;
	}

}

void test() {
	//g_maps.renewAbsorbing(31, 0, Vec2(0, 1), g_mDrip);

	/*test self*/
	//g_maps.renewAbsorbing(31, 1, Vec2(0, 0), g_mDrip);
	//g_maps.renewAbsorbing(31, 0, Vec2(0, 1), g_mDrip);

	/*test side*/
//	g_maps.renewAbsorbing(31, 15, Vec2(0, 0), g_mDrip);
//	g_maps.renewAbsorbing(15, 0, Vec2(0, 0), g_mDrip);

	/*test: compare row and col, row frist*/
	//g_maps.renewAbsorbing(15, 15, Vec2(0, 0), g_mDrip);

	/*test: compare row and col, col frist*/
	//g_maps.renewAbsorbing(15, 14, Vec2(0, 1), g_mDrip);

	/*test level*/
	//g_maps.renewAbsorbing(15, 15, Vec2(0, 1), g_mDrip, 0);


	/*test Linen*/
	//g_maps.renewAbsorbing(15, 15, Vec2(0, 1), g_mDrip);

	//UpdateSaturations(0, Vec2(0, 0));
	//UpdateSaturations(1, Vec2(0, 1));
	//UpdateSaturations(31, Vec2(1, 1));
	//UpdateSaturations(100, Vec2(1, 0));

	//float tmp = g_mDrip / 10;
	//for (int i = 0; i < 32; i++) {
	//	for (int j = 0; j < 32; j++) {
	//		g_maps.renewAbsorbing(i, j, Vec2(0, 0), tmp);
	//		g_maps.renewAbsorbing(i, j, Vec2(0, 1), tmp);
	//		g_maps.renewAbsorbing(i, j, Vec2(1, 0), tmp);
	//		g_maps.renewAbsorbing(i, j, Vec2(1, 1), tmp);
	//	}
	//}

	//float tmp = g_mDrip / 10;
	//for (int i = 0; i < 32; i++) {
	//	for (int j = 0; j < 32; j++) {
	//		if (i > 8 && i < 24 && j > 4 && j < 28) continue;
	//		g_maps.renewAbsorbing(i, j, Vec2(0, 0), tmp);
	//		g_maps.renewAbsorbing(i, j, Vec2(0, 1), tmp);
	//	}
	//}


	//float tmp = g_mDrip / 10;
	//for (int i = 8; i < 24; i++) {
	//	for (int j = 4; j < 28; j++) {
	//		g_maps.renewAbsorbing(i, j, Vec2(0, 0), tmp);
	//		g_maps.renewAbsorbing(i, j, Vec2(0, 1), tmp);
	//		g_maps.renewAbsorbing(i, j, Vec2(1, 0), tmp);
	//		g_maps.renewAbsorbing(i, j, Vec2(1, 1), tmp);
	//	}
	//}


	//float tmp = g_mDrip / 20;
	//int i = 31;
	//for (int j = 0; j < 32; j++) {
	//	g_maps.renewAbsorbing(i, j, Vec2(0, 0), tmp);
	//	g_maps.renewAbsorbing(i, j, Vec2(0, 1), tmp);
	//	g_maps.renewAbsorbing(i, j, Vec2(1, 0), tmp);
	//	g_maps.renewAbsorbing(i, j, Vec2(1, 1), tmp);
	//}

	float tmp = g_mDrip / 20;
	int j = 31;
	for (int i = 0; i < 32; i++) {
		g_maps.renewAbsorbing(i, j, Vec2(0, 0), tmp);
		g_maps.renewAbsorbing(i, j, Vec2(0, 1), tmp);
		g_maps.renewAbsorbing(i, j, Vec2(1, 0), tmp);
		g_maps.renewAbsorbing(i, j, Vec2(1, 1), tmp);
	}
	

}

void Absorbing() {

	int activeCount = flexGetActiveCount(g_flex);

	//float sum[4] = {0, 0, 0, 0};

	int i = g_numSolidParticles;
	while (i < activeCount) {
		int collidePosition = -1;
		for (int j = 0; j < g_numSolidParticles; j++) {
			if (Collide(i, j)) {
				collidePosition = j;
				break;
			}
		}

		//i is absorbable & collided
		if (g_absorbable[i] && collidePosition > -1) {

			//int t = pos.x + pos.y * 2;
			//sum[t]++;

			//proportion
			int tmp = rand() % 10;
			if (tmp >= 10 * g_kAbsorption) {
				i++;
				continue;
			}

			//cloth position j
			UpdateSaturations(collidePosition);

			//fluid point i
			g_positions[i] = g_positions[activeCount - 1];
			//g_positions[activeCount - 1] = Vec4();

			g_velocities[i] = g_velocities[activeCount - 1];
			g_velocities[activeCount - 1] = 0.0f;
			//delete &g_velocities[activeCount - 1];

			g_phases[i] = g_phases[activeCount - 1];
			g_phases[activeCount] = 0;
			//delete &g_phases[activeCount - 1];

			activeCount--;
			continue;
		}

		if (!g_absorbable[i] && collidePosition == -1) {
			g_absorbable[i] = true;
		}

		i++;

		flexSetActive(g_flex, &g_activeIndices[0], activeCount, eFlexMemoryHost);
	}
	//int tmp = activeCount - g_numSolidParticles;
	//if (tmp > 10) {
	//	std::cout << activeCount - g_numSolidParticles << ' ' << sum[0] << ' ' << sum[1] << ' ' << sum[2] << ' ' << sum[3] << std::endl;
	//}
}


//diffuse
void CalculateTriangleCenters() {
	if (g_triangleCenters.size() == 0) {
		g_triangleCenters.resize(g_numTriangles);
	}

	for (int i = 0; i < g_numTriangles; i++) {
		Vec3 points = g_trianglePoints[i];
		Vec3 position0 = g_positions[int(points.x)];
		Vec3 position1 = g_positions[int(points.y)];
		Vec3 position2 = g_positions[int(points.z)];

		//g_triangleCenters[i] = position1 + (position0 - position1 + position2 - position1) / 3;
		g_triangleCenters[i] = (position0 + position1 + position2) / 3;
	}
}
Vec3 calculateCosTheta(int index) {
	float t0 = 10.0, t1 = 10.0, t2 = 10.0;
	Vec3 neighbours = g_triangleNeighbours[index];
	if (neighbours.x >= 0) {
		Vec3 dir = g_triangleCenters[int(neighbours.x)] - g_triangleCenters[index];
		t0 = (-dir.y) / (sqrt(sqr(dir.x) + sqr(dir.y) + sqr(dir.z)));
	}
	if (neighbours.y >= 0) {
		Vec3 dir = g_triangleCenters[int(neighbours.y)] - g_triangleCenters[index];
		t1 = (-dir.y) / (sqrt(sqr(dir.x) + sqr(dir.y) + sqr(dir.z)));
	}
	if (neighbours.z >= 0) {
		Vec3 dir = g_triangleCenters[int(neighbours.z)] - g_triangleCenters[index];
		t2 = (-dir.y) / (sqrt(sqr(dir.x) + sqr(dir.y) + sqr(dir.z)));
	}
	return Vec3(t0, t1, t2);
}
float calculateCosTheta(int i, int x) {

	if (x < 0 || x >= g_numPoints) return 10.0;

	int xi = i / g_dy;
	int yi = i % g_dy;
	int xx = x / g_dy;
	int yx = x % g_dy;
	if (std::abs(xx - xi) + std::abs(yx - yi) > 1) return 10.0;

	Vec3 dir = g_positions[x] - g_positions[i];

	//if (i == 31 * 32) {
	//	std::cout << x << " : " << dir[0] << ' ' << dir[1] << ' ' << dir[2]  << std::endl;
	//}

	return (-dir.y) / (sqrt(sqr(dir.x) + sqr(dir.y) + sqr(dir.z)));
}
void CalculateTriangleThetas() {
	if (g_triangleThetas.size() == 0) {
		g_triangleThetas.resize(g_numTriangles);
	}

	for (int i = 0; i < g_numTriangles; i++) {
		g_triangleThetas[i] = calculateCosTheta(i);
	}


}
void CalculatePointThetas() {
	//pointThetas
	for (int i = 0; i < g_numPoints; i++) {
		Vec4 thetas;
		thetas[0] = calculateCosTheta(i, i - 1);	//right
		thetas[1] = calculateCosTheta(i, i + 1);	//left
		thetas[2] = calculateCosTheta(i, i - g_dy);	//up
		thetas[3] = calculateCosTheta(i, i + g_dy);	//down

		if (thetas[0] == 10) thetas[0] = -thetas[1];
		if (thetas[1] == 10) thetas[1] = -thetas[0];
		if (thetas[2] == 10) thetas[2] = -thetas[3];
		if (thetas[3] == 10) thetas[3] = -thetas[2];

		//if (i == 31 * 32) {
		//	std::cout << thetas[0] << ' ' << thetas[1] << ' ' << thetas[2] << ' ' << thetas[3] << std::endl;
		//}

		g_maps.renewPointTheta(i, thetas);

	}
}

void Diffusing() {
	vector<float> deltas;
	deltas.resize(g_numTriangles);

	for (int i = 0; i < g_numTriangles; i++) {
		float sSum = 0;
		float si = g_saturations[i];
		Vec3 thetas = g_triangleThetas[i];
		Vec3 neighbours = g_triangleNeighbours[i];
		Vec3 deltasin = Vec3(0.0, 0.0, 0.0);

		if (thetas.x <= 1.0 && neighbours.x >= 0) {
			deltasin.x = getMax(0.0f, g_kDiffusion * (si - g_saturations[int(neighbours.x)]) + g_kDiffusionGravity * si * thetas.x);
			sSum += deltasin.x;
		}
		if (thetas.y <= 1.0 && neighbours.y >= 0) {
			deltasin.y = getMax(0.0f, g_kDiffusion * (si - g_saturations[int(neighbours.y)]) + g_kDiffusionGravity * si * thetas.y);
			sSum += deltasin.y;
		}
		if (thetas.z <= 1.0 && neighbours.z >= 0) {
			deltasin.z = getMax(0.0f, g_kDiffusion * (si - g_saturations[int(neighbours.z)]) + g_kDiffusionGravity * si * thetas.z);
			sSum += deltasin.z;
		}

		float normFac = 1.0;
		if (sSum > si) {
			normFac = si / sSum;
		}

		if (thetas.x <= 1.0 && neighbours.x >= 0) {
			deltas[i] += -normFac * deltasin.x;
			deltas[int(neighbours.x)] += normFac * deltasin.x;
		}
		if (thetas.y <= 1.0 && neighbours.y >= 0) {
			deltas[i] += -normFac * deltasin.y;
			deltas[int(neighbours.y)] += normFac * deltasin.y;
		}
		if (thetas.z <= 1.0 && neighbours.z >= 0) {
			deltas[i] += -normFac * deltasin.z;
			deltas[int(neighbours.z)] += normFac * deltasin.z;
		}
	}
	for (int i = 0; i < g_numTriangles; i++) {
		g_saturations[i] += deltas[i];
		if (g_saturations[i] < 0) {
			if (g_saturations[i] == -0.0) {
				printf("mark\n");
			}
			g_saturations[i] = 0;
		}
	}

	//g_maps.renewDiffusing(g_kDiffusion, g_kDiffusionGravity);

}

//drip
void CreateParticle(int index, int &activeCount) {
	Vec3 emitterDir = Vec3(0.0f, -1.0f, 0.0f);
	Vec3 emitterRight = Vec3(-1.0f, 0.0f, 0.0f);

	//position
	Vec3 centerPos = g_triangleCenters[index];
	Vec3 aPos = g_positions[int(g_trianglePoints[index].x)];
	Vec3 bPos = g_positions[int(g_trianglePoints[index].y)];
	Vec3 cPos = g_positions[int(g_trianglePoints[index].z)];

	int a = rand() % 101;
	int b = rand() % 101;
	int c = rand() % 101;
	Vec3 emitterPos = (centerPos * (101 - a - b - c) + aPos * a + bPos * b + cPos * c) / 100.0;
	emitterPos -= g_triangleNormals[index] * g_params.mCollisionDistance * 2.0;

	float r;
	int phase;

	if (g_params.mFluid) {
		r = g_params.mFluidRestDistance;
		phase = flexMakePhase(0, eFlexPhaseSelfCollide | eFlexPhaseFluid);
	}
	else {
		r = g_params.mSolidRestDistance;
		phase = flexMakePhase(0, eFlexPhaseSelfCollide);
	}

	if (size_t(activeCount) < g_positions.size()) {
		g_positions[activeCount] = Vec4(emitterPos, 1.0f);
		g_velocities[activeCount] = Vec3(0.0f, 0.0f, 0.0f);
		g_phases[activeCount] = phase;
		g_absorbable[activeCount] = true;
		activeCount++;
	}
}

void Dripping() {
	if (g_dripBuffer.size() == 0) {
		g_dripBuffer.resize(g_numTriangles);
	}

	int triangleNum = (g_dx - 1) * (g_dy - 1) * 2;

	int activeCount = flexGetActiveCount(g_flex);

	float maxSaturation = g_maxSaturation * g_kMaxAbsorption;

	for (int i = 0; i < triangleNum; i++) {
		if (g_saturations[i] > maxSaturation) {
			float m = g_saturations[i] - maxSaturation;
			g_dripBuffer[i] += m;
			while (g_dripBuffer[i] > g_mDrip) {
				CreateParticle(i, activeCount);
				g_dripBuffer[i] -= g_mDrip;
			}
			g_saturations[i] = maxSaturation;
		}
	}

	flexSetActive(g_flex, &g_activeIndices[0], activeCount, eFlexMemoryHost);
}

/*add end*/

