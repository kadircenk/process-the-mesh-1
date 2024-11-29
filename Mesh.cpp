#include "Mesh.h"
#include "Util.h"

Mesh::Mesh(const char* name) {
	if (!strcmp(Util::getFileExt(name), "obj")) //argv[1] is mesh file path /w extension .off or .obj
		loadObj(name);
	else //.off extension
		loadOff(name);
	calculateEdgeLengths(); //set edge lengths for geodesic distance calculation
};

void Mesh::calculateEdgeLengths() {
	int edgesSize = edges.size();
	for (int i = 0; i < edgesSize; i++) {
		edges[i]->length = sqrtf(powf(verts[edges[i]->v1i]->coords[0] - verts[edges[i]->v2i]->coords[0], 2.0f)
			+ powf(verts[edges[i]->v1i]->coords[1] - verts[edges[i]->v2i]->coords[1], 2.0f)
			+ powf(verts[edges[i]->v1i]->coords[2] - verts[edges[i]->v2i]->coords[2], 2.0f));
	}
}

void Mesh::loadOff(const char* name)
{
	FILE* fPtr;
	fopen_s(&fPtr, name, "r");
	if (!fPtr)perror("fopen");

	char str[334];

	fscanf_s(fPtr, "%s", str, 4); //OFF

	int nVerts, nTris, n, i = 0;
	float x, y, z;

	fscanf_s(fPtr, "%d %d %d\n", &nVerts, &nTris, &n);
	while (i++ < nVerts)
	{
		fscanf_s(fPtr, "%f %f %f", &x, &y, &z);
		addVertex(x, y, z);
	}

	while (fscanf_s(fPtr, "%d", &i) != EOF)
	{
		fscanf_s(fPtr, "%f %f %f", &x, &y, &z);
		addTriangle((int)x, (int)y, (int)z);
	}

	fclose(fPtr);
}


void Mesh::loadObj(const char* name)
{
	FILE* fPtr;
	fopen_s(&fPtr, name, "r");
	if (!fPtr)perror("fopen");

	char str[334];

	fscanf_s(fPtr, "%s", str, 4); //#
	fscanf_s(fPtr, "%s", str, 4); //OBJ

	int nVerts = 0, nTris = 0, n = 0, i = 0;
	float x, y, z;

	fscanf_s(fPtr, "%s", str, 2);

	while (str[0] == 'v')
	{
		fscanf_s(fPtr, "%f %f %f\n", &x, &y, &z);
		addVertex(x, y, z);

		fscanf_s(fPtr, "%s", str, 2);
		nVerts++;
	}

	int k = 0;
	while (str[0] == 'f')
	{
		k++;
		fscanf_s(fPtr, "%f %f %f\n", &x, &y, &z);
		addTriangle((int)x - 1, (int)y - 1, (int)z - 1);//-1ler var OBJ icin

		if (fscanf_s(fPtr, "%s", str, 2) == EOF)
			break;
	}

	fclose(fPtr);
}


void Mesh::addTriangle(int v1, int v2, int v3)
{
	int idx = tris.size();
	tris.push_back(new Triangle(idx, v1, v2, v3));

	//set up structure

	verts[v1]->triList.push_back(idx);
	verts[v2]->triList.push_back(idx);
	verts[v3]->triList.push_back(idx);

	if (!makeVertsNeighbor(v1, v2))
		addEdge(v1, v2);

	if (!makeVertsNeighbor(v1, v3))
		addEdge(v1, v3);

	if (!makeVertsNeighbor(v2, v3))
		addEdge(v2, v3);

}

bool Mesh::makeVertsNeighbor(int v1i, int v2i)
{
	//returns true if v1i already neighbor w/ v2i; false o/w

	for (int i = 0; i < verts[v1i]->vertList.size(); i++)
		if (verts[v1i]->vertList[i] == v2i)
			return true;


	verts[v1i]->vertList.push_back(v2i);
	verts[v2i]->vertList.push_back(v1i);
	return false;
}

void Mesh::addVertex(float x, float y, float z)
{
	int idx = verts.size();
	float* c = new float[3];
	c[0] = x;
	c[1] = y;
	c[2] = z;

	verts.push_back(new Vertex(idx, c));
}

void Mesh::addEdge(int v1, int v2)
{
	int idx = edges.size();

	edges.push_back(new Edge(idx, v1, v2));

	verts[v1]->edgeList.push_back(idx);
	verts[v2]->edgeList.push_back(idx);
}
