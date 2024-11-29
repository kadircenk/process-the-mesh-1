#define HAVE_INT8_T
#include <Inventor/Win/SoWin.h>
#include <Inventor/Win/viewers/SoWinExaminerViewer.h>
#include <Inventor/Win/viewers/SoWinViewer.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoDrawStyle.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoIndexedFaceSet.h>
#include <Inventor/nodes/SoIndexedLineSet.h>
#include <Inventor/nodes/SoShapeHints.h>

#include <boost/heap/fibonacci_heap.hpp>

#include <time.h>
#include <queue>

#include "Mesh.h"
#include "Util.h"

int dataStructure;
float sphereScale;

struct MinHeapVertex
{
	Vertex* v;
	float pushAnindakiDist;

	boost::heap::fibonacci_heap<MinHeapVertex>::handle_type handle;

	MinHeapVertex(Vertex* ver) {
		v = ver;
		pushAnindakiDist = ver->curGeoDist;
	}
};

//struct CompareFiboNode;

struct heap_data
{
	Vertex* v;
	boost::heap::fibonacci_heap<heap_data>::handle_type handle;

	heap_data(Vertex* i) :
		v(i)
	{}

	bool operator<(heap_data const& rhs) const
	{
		return v->curGeoDist > rhs.v->curGeoDist;
	}
};

//struct CompareFiboNode
//{
//	bool operator()(heap_data l, heap_data r) const
//	{
//		return l.curFiboGeoDist > r.curFiboGeoDist;
//	}
//
//	/*bool operator<(heap_data const& rhs) const
//	{
//		return curFiboGeoDist < rhs.curFiboGeoDist;
//	}*/
//	/*bool operator()(Vertex* l, Vertex* r) const
//	{
//		return l->curGeoDist > r->curGeoDist;
//	}*/
//};

void relaxDijkstra(Mesh* mesh, Vertex* minVert) {
	for (size_t j = 0; j < minVert->vertList.size(); j++)
	{
		float wEdgeCost = 0;

		for (int ne = 0; ne < minVert->edgeList.size(); ne++) {
			//minVert ile su anki komsusu arasindaki Edge'i bul ve wEdgeCost'u bu Edge'in length'ine esitle.
			if ((mesh->edges[minVert->edgeList[ne]]->v1i == minVert->idx && mesh->edges[minVert->edgeList[ne]]->v2i == mesh->verts[minVert->vertList[j]]->idx)
				|| (mesh->edges[minVert->edgeList[ne]]->v2i == minVert->idx && mesh->edges[minVert->edgeList[ne]]->v1i == mesh->verts[minVert->vertList[j]]->idx)) {
				wEdgeCost = mesh->edges[minVert->edgeList[ne]]->length;
				break;
			}
		}

		//do relax, also set parent for red thick path drawing between two query pts.
		if (mesh->verts[minVert->vertList[j]]->curGeoDist > minVert->curGeoDist + wEdgeCost) {
			mesh->verts[minVert->vertList[j]]->curGeoDist = minVert->curGeoDist + wEdgeCost;
			mesh->verts[minVert->vertList[j]]->curGeoParent = minVert;
		}
	}
}


vector<float> calculateDijkstra(Mesh* mesh, Vertex* vertex) {
	//dataStructure: 0->array, 1->min heap, 2->fibonacci heap. 
	vector<float> returnSatiri; //1xN, ben tek siz hepiniz diyor vertex param.

	//select the required data structure
	if (dataStructure == 0) //array
	{
		vector<Vertex*> tumuQ;

		//init dijkstra costs, 0 for vertex param, nearly infinity for other N-1 verts
		for (size_t i = 0; i < mesh->verts.size(); i++)
		{
			mesh->verts[i]->curProcessed = false;
			mesh->verts[i]->curGeoDist = 99999.0f;
			mesh->verts[i]->curGeoParent = NULL;
			tumuQ.push_back(mesh->verts[i]);
		}
		vertex->curGeoDist = 0.0f;

		for (size_t i = 0; i < tumuQ.size(); i++)
		{
			int minId = 99999;
			float minDist = 99999.0f;
			Vertex* minVert = vertex;

			//extract min from array O(N)
			for (size_t j = 0; j < tumuQ.size(); j++)
			{
				if (!(tumuQ[j]->curProcessed) && tumuQ[j]->curGeoDist < minDist) {
					minId = j;
					minDist = tumuQ[j]->curGeoDist;
					minVert = tumuQ[j];
				}
			}

			//minVert islendi olarak isaretle
			minVert->curProcessed = true;

			//komsularini relax et
			relaxDijkstra(mesh, minVert);
		}
	}
	else if (dataStructure == 1) //min-heap
	{
		//min-heap structure tutorial at https://stackoverflow.com/questions/2786398/is-there-an-easy-way-to-make-a-min-heap-in-c
		//custom comparator tutorial at https://stackoverflow.com/questions/16111337/declaring-a-priority-queue-in-c-with-a-custom-comparator/48587737
		auto compare = [](MinHeapVertex l, MinHeapVertex r) { return l.pushAnindakiDist > r.pushAnindakiDist; };
		priority_queue<MinHeapVertex, std::vector<MinHeapVertex>, decltype(compare)> tumuQ(compare);

		//init dijkstra costs, 0 for vertex param, nearly infinity for other N-1 verts
		for (size_t i = 0; i < mesh->verts.size(); i++)
		{
			mesh->verts[i]->curProcessed = false;
			mesh->verts[i]->curGeoDist = 99999.0f;
			mesh->verts[i]->curGeoParent = NULL;
		}

		vertex->curGeoDist = 0.0f;

		/*for (size_t i = 0; i < mesh->verts.size(); i++)
		{
			tumuQ.push(MinHeapVertex(mesh->verts[i]));
		}*/
		tumuQ.push(MinHeapVertex(vertex));

		while (!tumuQ.empty()) {
			Vertex* minVert = tumuQ.top().v; //extract min from min-heap
			tumuQ.pop();

			//islenmisse devam et
			if (minVert->curProcessed)
				continue;

			//simdi isliyorum
			minVert->curProcessed = true;

			// relax for each adj vert
			relaxDijkstra(mesh, minVert);

			// min-heapteki elemanlar relax oldu, fakat min-heap'teki vertex pozisyonlari guncellenmedi.
			// so, we add new copies of those updated ones, and also keeping their first copies in heap.
			// idea from: https://stackoverflow.com/questions/9209323/easiest-way-of-using-min-priority-queue-with-key-update-in-c
			// eski kopyalar zaten ustteki if (minVert->curProcessed) checki ile atlaniyor.
			for (size_t i = 0; i < minVert->vertList.size(); i++)
			{
				//islenmemisse ekle.
				if (!(mesh->verts[minVert->vertList[i]]->curProcessed)) {
					tumuQ.push(MinHeapVertex(mesh->verts[minVert->vertList[i]]));
				}
			}
		}
	}
	else if (dataStructure == 2) //fibo heap
	{
		boost::heap::fibonacci_heap<heap_data> fiboHeap;

		//init dijkstra costs, 0 for vertex param, nearly infinity for other N-1 verts
		for (size_t i = 0; i < mesh->verts.size(); i++)
		{
			mesh->verts[i]->curProcessed = false;
			mesh->verts[i]->curGeoDist = 99999.0f;
			mesh->verts[i]->curGeoParent = NULL;

		}

		vertex->curGeoDist = 0.0f;

		fiboHeap.push(heap_data(vertex));

		while (!fiboHeap.empty()) {
			Vertex* minVert = fiboHeap.top().v; //extract min from fibo-heap
			fiboHeap.pop();

			//islenmisse devam et
			if (minVert->curProcessed)
				continue;

			//simdi isliyorum
			minVert->curProcessed = true;

			// relax for each adj vert
			relaxDijkstra(mesh, minVert);

			for (size_t i = 0; i < minVert->vertList.size(); i++)
			{
				//islenmemisse ekle.
				if (!(mesh->verts[minVert->vertList[i]]->curProcessed)) {
					fiboHeap.push(heap_data(mesh->verts[minVert->vertList[i]]));
				}
			}
		}
	}

	//geo dists are ready. get geo dist of current vertex param to all other N-1 verts as 1 vector line and return it.
	for (size_t i = 0; i < mesh->verts.size(); i++) {
		returnSatiri.push_back(mesh->verts[i]->curGeoDist);
	}

	////calculate AGD if requested
	//if (calcAGD) {
	//	float sum = 0;
	//	for (size_t i = 0; i < returnSatiri.size(); i++)
	//		sum += returnSatiri[i];

	//	vertex->avgGeoDist = sum / returnSatiri.size();

	//	if (vertex->avgGeoDist > mesh->maxAvgGeoDist) //set maxAvgGeoDist of mesh
	//		mesh->maxAvgGeoDist = vertex->avgGeoDist;
	//}

	return returnSatiri;
}

SoSeparator* getSpheresSep(Mesh* mesh, float deltaX, float deltaY, float scale, float radius)
{
	//returns a set of spheres to highlight each mesh.samples[i]

	SoSeparator* spheresSep = new SoSeparator();

	for (int i = 0; i < (int)mesh->samples.size(); i++)
	{
		//1 sphere for this sample
		SoSeparator* sphere1Sep = new SoSeparator;

		//transformation
		SoTransform* tra = new SoTransform();
		tra->translation.setValue(scale * mesh->verts[mesh->samples[i]]->coords[0] + deltaX, scale * mesh->verts[mesh->samples[i]]->coords[1] + deltaY, scale * mesh->verts[mesh->samples[i]]->coords[2]);
		sphere1Sep->addChild(tra);

		//material
		SoMaterial* ma = new SoMaterial;
		ma->diffuseColor.setValue(SbColor(0.7f, 0.0f, 0.0f));
		/*if (i == 0)
			ma->diffuseColor.setValue(SbColor(0.0f, 0.0f, 0.7f));
		else if (i == 1)
			ma->diffuseColor.setValue(SbColor(0.7f, 0.7f, 0.2f));
		else if (i == 2)
			ma->diffuseColor.setValue(SbColor(0.0f, 0.7f, 0.0f));
		else if (i == 3)
			ma->diffuseColor.setValue(SbColor(0.7f, 0.0f, 0.7f));
		else if (i == 4)
			ma->diffuseColor.setValue(SbColor(0.7f, 0.7f, 0.0f));
		else
			ma->diffuseColor.setValue(SbColor(0.7f, 0.0f, 0.0f));*/

		sphere1Sep->addChild(ma);

		//shape
		SoSphere* sph1 = new SoSphere();
		sph1->radius = radius;
		sphere1Sep->addChild(sph1); //whose position is decided by the translation applied above

		spheresSep->addChild(sphere1Sep);
	}

	return spheresSep;
}

SoSeparator* getShapeSep(Mesh* mesh, int mission, vector<int> geoDistQueryPathVertIds)
{
	SoSeparator* res = new SoSeparator();
	if (mission == 0 || mission == 1) {
		SoMaterial* mat = new SoMaterial;
		mat->diffuseColor.set1Value(0, 0.8, 0.8, 0.8); //paint all vertices with this color
		mat->diffuseColor.set1Value(1, 0, 1, 0); //paint all vertices with this color
		mat->diffuseColor.set1Value(2, 1, 0, 0); //paint all vertices with this color
		res->addChild(mat);

		//SoShapeHints* hints = new SoShapeHints;
		//hints->creaseAngle = 3.14f;
		//res->addChild(hints); //Gouraud shading

		SoMaterialBinding* materialBinding = new SoMaterialBinding; //for 2+ diffuse color usage on the same mesh
		materialBinding->value = SoMaterialBinding::PER_VERTEX_INDEXED;
		res->addChild(materialBinding);

		SoCoordinate3* coords = new SoCoordinate3();
		for (int c = 0; c < mesh->verts.size(); c++)
			coords->point.set1Value(c, mesh->verts[c]->coords[0], mesh->verts[c]->coords[1], mesh->verts[c]->coords[2]);
		res->addChild(coords);

		SoIndexedFaceSet* faceSet = new SoIndexedFaceSet();
		for (int c = 0; c < mesh->tris.size(); c++)
		{
			faceSet->coordIndex.set1Value(c * 4, mesh->tris[c]->v1i);
			faceSet->coordIndex.set1Value(c * 4 + 1, mesh->tris[c]->v2i);
			faceSet->coordIndex.set1Value(c * 4 + 2, mesh->tris[c]->v3i);
			faceSet->coordIndex.set1Value(c * 4 + 3, -1);

			faceSet->materialIndex.set1Value(c * 4, 0);
			faceSet->materialIndex.set1Value(c * 4 + 1, 0);
			faceSet->materialIndex.set1Value(c * 4 + 2, 0);
			faceSet->materialIndex.set1Value(c * 4 + 3, -1);
		}
		res->addChild(faceSet);
	}
	if (mission == 1) {

		SoDrawStyle* draw = new SoDrawStyle;
		draw->lineWidth.setValue(4);
		res->addChild(draw);

		SoIndexedLineSet* lineSet = new SoIndexedLineSet();
		for (int c = 0; c < geoDistQueryPathVertIds.size(); c++)
		{
			lineSet->coordIndex.set1Value(c, geoDistQueryPathVertIds[c]);
			lineSet->materialIndex.set1Value(c, 2);
		}
		lineSet->coordIndex.set1Value(geoDistQueryPathVertIds.size(), -1);
		lineSet->materialIndex.set1Value(geoDistQueryPathVertIds.size(), -1);

		res->addChild(lineSet);
	}
	if (mission == 2) { //AGD boyama
		SoMaterial* mat = new SoMaterial;

		int imax = 100;
		for (size_t i = 0; i < imax; i++)
			mat->diffuseColor.set1Value(i, i * (1.0f / (imax - 1)), (imax / 2.0f) - abs(i - imax / 2.0f), ((imax - 1) - i) * (1.0f / (imax - 1)));
		//mat->diffuseColor.set1Value(i, i / (imax / 3) == 0 ? 1.0 : 0, i / (imax / 3) == 1 ? 1.0 : 0, i / (imax / 3) == 2 ? 1.0 : 0);
		mat->diffuseColor.set1Value(imax, 1.0, 0, 0);

		res->addChild(mat);

		SoMaterialBinding* materialBinding = new SoMaterialBinding; //for 2+ diffuse color usage on the same mesh
		materialBinding->value = SoMaterialBinding::PER_VERTEX_INDEXED;
		res->addChild(materialBinding);

		SoCoordinate3* coords = new SoCoordinate3();
		for (int c = 0; c < mesh->verts.size(); c++)
			coords->point.set1Value(c, mesh->verts[c]->coords[0], mesh->verts[c]->coords[1], mesh->verts[c]->coords[2]);
		res->addChild(coords);

		SoIndexedFaceSet* faceSet = new SoIndexedFaceSet();
		for (int c = 0; c < mesh->tris.size(); c++)
		{
			faceSet->coordIndex.set1Value(c * 4, mesh->tris[c]->v1i);
			faceSet->coordIndex.set1Value(c * 4 + 1, mesh->tris[c]->v2i);
			faceSet->coordIndex.set1Value(c * 4 + 2, mesh->tris[c]->v3i);
			faceSet->coordIndex.set1Value(c * 4 + 3, -1);

			faceSet->materialIndex.set1Value(c * 4, (int)((mesh->verts[mesh->tris[c]->v1i]->avgGeoDist - mesh->minAvgGeoDist)
				/ (mesh->maxAvgGeoDist - mesh->minAvgGeoDist) * imax));
			faceSet->materialIndex.set1Value(c * 4 + 1, (int)((mesh->verts[mesh->tris[c]->v2i]->avgGeoDist - mesh->minAvgGeoDist)
				/ (mesh->maxAvgGeoDist - mesh->minAvgGeoDist) * imax));
			faceSet->materialIndex.set1Value(c * 4 + 2, (int)((mesh->verts[mesh->tris[c]->v3i]->avgGeoDist - mesh->minAvgGeoDist)
				/ (mesh->maxAvgGeoDist - mesh->minAvgGeoDist) * imax));

			faceSet->materialIndex.set1Value(c * 4 + 3, -1);
		}
		res->addChild(faceSet);
	}

	return res;
}

void solveQ1(SoSeparator* root, Mesh* mesh) {
	int task;
	printf_s("Task (0: find & write M to file, 1: draw two-query-pt GD path, 2: show avg. timing result of 10 GD run of first 10 vertices): ");
	scanf_s("%d", &task);

	if (task == 0) {
		char filePath[300];
		printf_s("Write M matrix to file path: ");
		scanf_s("%s", filePath, 300);

		vector<vector<float>> tumGeodesicDistances;
		for (size_t i = 0; i < mesh->verts.size(); i++)
			tumGeodesicDistances.push_back(calculateDijkstra(mesh, mesh->verts[i]));

		Util::writeMatrixToFile(tumGeodesicDistances, filePath); //write matrix M to file
		//printf_s("M matrix is written to the provided path. Showing the mesh visual...\n");
		root->addChild(getShapeSep(mesh, 0, vector<int>())); //meshi ekrana bos bas
	}
	else if (task == 1) {
		vector<int> geoDistQueryPathVertIds;
		int startVertId; //geo dist path start vert id
		int endVertId; //geo dist path end vert id

		printf_s("Start vertex index: ");
		scanf_s("%d", &startVertId);

		printf_s("End vertex index: ");
		scanf_s("%d", &endVertId);

		calculateDijkstra(mesh, mesh->verts[startVertId]);

		//simdi mesh'in icinde vertexlerde az onceki Dijkstra sonuclari hep hazir, parent info ve dist'ler yani.
		Vertex* curV = mesh->verts[endVertId];
		while (curV->curGeoParent != NULL)
		{
			geoDistQueryPathVertIds.push_back(curV->idx);
			curV = curV->curGeoParent;
		}
		geoDistQueryPathVertIds.push_back(startVertId);

		root->addChild(getShapeSep(mesh, 1, geoDistQueryPathVertIds));
	}
	else if (task == 2) {
		double total_time_taken = 0;
		clock_t t;
		for (size_t i = 0; i < 10; i++) {
			t = clock();

			calculateDijkstra(mesh, mesh->verts[i]);

			t = clock() - t; //calculate time elapsed
			total_time_taken += ((double)t) / CLOCKS_PER_SEC; // calculate the elapsed time
		}
		printf("Launched 10 Dijkstra calculations, each for first 10 vertices, Time elapsed: %f seconds.\n\n", total_time_taken);

		root->addChild(getShapeSep(mesh, 0, vector<int>()));
	}
}

void solveQ2(SoSeparator* root, Mesh* mesh) {
	float max = -1;
	Vertex* vert;
	int vertId;
	int totalVerts = mesh->verts.size();

	//find vertex /w max'ing AGD, but too slow for high-reso models. we used random selected V, and used its max geoList V as first FPS V.
	/*for (size_t i = 0; i < totalVerts; i++)
	{
		mesh->verts[i]->geoDistList = calculateDijkstra(mesh, mesh->verts[i], atoi(argv[3]));
		float sum = 0;
		for (size_t j = 0; j < totalVerts; j++)
		{
			sum += mesh->verts[i]->geoDistList[j];
		}
		if (max <= sum) {
			max = sum;
			vert = mesh->verts[i];
			vertId = i;
		}
	}*/
	mesh->verts[0]->geoDistList = calculateDijkstra(mesh, mesh->verts[0]);
	for (size_t u = 0; u < mesh->verts[0]->geoDistList.size(); u++)
	{
		if (max <= mesh->verts[0]->geoDistList[u]) {
			max = mesh->verts[0]->geoDistList[u];
			vert = mesh->verts[u];
			vertId = u;
		}
	}
	//found vert max'ing GD

	vector<Vertex*> FpsSamples;

	mesh->verts[vertId]->geoDistList = calculateDijkstra(mesh, mesh->verts[vertId]);
	FpsSamples.push_back(vert);

	//find 9 more farthest pt samples
	for (size_t i = 0; i < 9; i++)
	{
		for (size_t j = 0; j < totalVerts; j++)
		{
			mesh->verts[j]->curMinDistFPS = 99999;

			for (size_t t = 0; t < FpsSamples.size(); t++)
			{
				if (FpsSamples[t]->geoDistList[j] < mesh->verts[j]->curMinDistFPS) {
					mesh->verts[j]->curMinDistFPS = FpsSamples[t]->geoDistList[j];
				}
			}
		}

		//find max of min candidates
		float max2 = -1;
		Vertex* vert2;
		int vertId2;
		for (size_t j = 0; j < totalVerts; j++)
		{
			if (mesh->verts[j]->curMinDistFPS > max2) {
				max2 = mesh->verts[j]->curMinDistFPS;
				vert2 = mesh->verts[j];
				vertId2 = j;
			}
		}

		mesh->verts[vertId2]->geoDistList = calculateDijkstra(mesh, mesh->verts[vertId2]);
		FpsSamples.push_back(vert2);
	}

	mesh->samples.clear();
	for (size_t j = 0; j < FpsSamples.size(); j++)
	{
		mesh->samples.push_back(FpsSamples[j]->idx);
	}
	root->addChild(getShapeSep(mesh, 0, vector<int>()));
	root->addChild(getSpheresSep(mesh, 0, 0, 1, sphereScale));
}

void solveQ3(SoSeparator* root, Mesh* mesh) {
	float max = -1;
	Vertex* vert;
	int vertId;
	int totalVerts = mesh->verts.size();

	mesh->verts[0]->geoDistList = calculateDijkstra(mesh, mesh->verts[0]);
	for (size_t u = 0; u < mesh->verts[0]->geoDistList.size(); u++)
	{
		if (max <= mesh->verts[0]->geoDistList[u]) {
			max = mesh->verts[0]->geoDistList[u];
			vert = mesh->verts[u];
			vertId = u;
		}
	}

	vector<Vertex*> FpsSamples;

	mesh->verts[vertId]->geoDistList = calculateDijkstra(mesh, mesh->verts[vertId]);
	FpsSamples.push_back(vert);

	//for high-res models' AGD calculation, 100 FPS points sampling.
	//find 99 more farthest pt samples, ysf hoca's tip for calculating AGD fast using FPS samples
	//for (size_t i = 0; i < 499; i++)
	//{
	//	for (size_t j = 0; j < totalVerts; j++)
	//	{
	//		mesh->verts[j]->curMinDistFPS = 99999;

	//		for (size_t t = 0; t < FpsSamples.size(); t++)
	//		{
	//			if (FpsSamples[t]->geoDistList[j] < mesh->verts[j]->curMinDistFPS) {
	//				mesh->verts[j]->curMinDistFPS = FpsSamples[t]->geoDistList[j];
	//			}
	//		}
	//	}

	//	//find max of min candidates
	//	float max2 = -1;
	//	Vertex* vert2;
	//	int vertId2;
	//	for (size_t j = 0; j < totalVerts; j++)
	//	{
	//		if (mesh->verts[j]->curMinDistFPS > max2) {
	//			max2 = mesh->verts[j]->curMinDistFPS;
	//			vert2 = mesh->verts[j];
	//			vertId2 = j;
	//		}
	//	}

	//	mesh->verts[vertId2]->geoDistList = calculateDijkstra(mesh, mesh->verts[vertId2]);
	//	FpsSamples.push_back(vert2);
	//}
	//100 FPS samples calculated, use them to approximate AGD fast

	vector<vector<float>> geoDists;
	for (size_t i = 0; i < mesh->verts.size(); i++)
		geoDists.push_back(calculateDijkstra(mesh, mesh->verts[i])); //calc geo dist /w AGD

	for (size_t i = 0; i < mesh->verts.size(); i++)
	{
		float sum = 0;
		for (size_t j = 0; j < mesh->verts.size(); j++)
			sum += geoDists[i][j];

		mesh->verts[i]->avgGeoDist = sum / mesh->verts.size();
	}

	//calc AGD of all verts
	//for (size_t i = 0; i < mesh->verts.size(); i++)
	//{
	//	float sum = 0;
	//	for (size_t j = 0; j < FpsSamples.size(); j++)
	//		sum += FpsSamples[j]->geoDistList[i];

	//	mesh->verts[i]->avgGeoDist = sum / FpsSamples.size();
	//}

	//triangle formula of paper, not used.
	//for (size_t i = 0; i < mesh->verts.size(); i++)
	//{
	//	float total1RingArea = 0;

	//	for (size_t t = 0; t < mesh->verts[i]->triList.size(); t++)
	//	{
	//		total1RingArea += Util::getTriangleArea(mesh->verts[mesh->tris[mesh->verts[i]->triList[t]]->v1i],
	//			mesh->verts[mesh->tris[mesh->verts[i]->triList[t]]->v2i],
	//			mesh->verts[mesh->tris[mesh->verts[i]->triList[t]]->v3i]);
	//	}

	//	mesh->verts[i]->avgGeoDist *= total1RingArea / 3.0f;
	//}

	//smooth out AGD
	for (size_t i = 0; i < mesh->verts.size(); i++)
	{
		float sum = mesh->verts[i]->avgGeoDist;
		for (size_t j = 0; j < mesh->verts[i]->vertList.size(); j++)
		{
			sum += mesh->verts[mesh->verts[i]->vertList[j]]->avgGeoDist;
		}
		mesh->verts[i]->avgGeoDist = sum / (1 + mesh->verts[i]->vertList.size());

		//set max and min AGD
		if (mesh->verts[i]->avgGeoDist > mesh->maxAvgGeoDist)
			mesh->maxAvgGeoDist = mesh->verts[i]->avgGeoDist;
		if (mesh->verts[i]->avgGeoDist < mesh->minAvgGeoDist)
			mesh->minAvgGeoDist = mesh->verts[i]->avgGeoDist;
	}


	vector<Vertex*> s1;
	for (size_t i = 0; i < mesh->verts.size(); i++) {
		int isLocalMax = 1;
		for (size_t j = 0; j < mesh->verts[i]->vertList.size(); j++)
		{
			if (mesh->verts[mesh->verts[i]->vertList[j]]->avgGeoDist >= mesh->verts[i]->avgGeoDist) {
				isLocalMax = 0;
			}
		}
		if (isLocalMax) {
			mesh->verts[i]->geoDistList = calculateDijkstra(mesh, mesh->verts[i]);
			s1.push_back(mesh->verts[i]);
		}
	}
	//use if local-minima needed
	/*for (size_t i = 0; i < mesh->verts.size(); i++) {
		int isLocalMin = 1;
		for (size_t j = 0; j < mesh->verts[i]->vertList.size(); j++)
		{
			if (mesh->verts[mesh->verts[i]->vertList[j]]->avgGeoDist <= mesh->verts[i]->avgGeoDist) {
				isLocalMin = 0;
			}
		}
		if (isLocalMin) {
			s1.push_back(mesh->verts[i]);
		}
	}*/

	for (size_t j = 0; j < totalVerts; j++)
	{
		mesh->verts[j]->curMinDistFPS = 99999;

		for (size_t t = 0; t < s1.size(); t++)
		{
			if (s1[t]->geoDistList[j] < mesh->verts[j]->curMinDistFPS) {
				mesh->verts[j]->curMinDistFPS = s1[t]->geoDistList[j];
			}
		}
	}

	//smooth out MGD
	for (size_t i = 0; i < mesh->verts.size(); i++)
	{
		float sum = mesh->verts[i]->curMinDistFPS;
		for (size_t j = 0; j < mesh->verts[i]->vertList.size(); j++)
		{
			sum += mesh->verts[mesh->verts[i]->vertList[j]]->curMinDistFPS;
		}
		mesh->verts[i]->curMinDistFPS = sum / (1 + mesh->verts[i]->vertList.size());
	}

	vector<Vertex*> s2;
	for (size_t i = 0; i < mesh->verts.size(); i++) {
		int isLocalMax = 1;
		for (size_t j = 0; j < mesh->verts[i]->vertList.size(); j++)
		{
			if (mesh->verts[mesh->verts[i]->vertList[j]]->curMinDistFPS >= mesh->verts[i]->curMinDistFPS) {
				isLocalMax = 0;
			}
		}
		if (isLocalMax) {
			s2.push_back(mesh->verts[i]);
		}
	}

	mesh->samples.clear();
	for (size_t j = 0; j < s1.size(); j++)
	{
		mesh->samples.push_back(s1[j]->idx);
	}
	for (size_t j = 0; j < s2.size(); j++)
	{
		mesh->samples.push_back(s2[j]->idx);
	}
	root->addChild(getShapeSep(mesh, 0, vector<int>()));
	root->addChild(getSpheresSep(mesh, 0, 0, 1, sphereScale));
}

int main(int, char** argv)
{
	HWND window = SoWin::init(argv[0]);
	if (window == NULL) exit(1);

	SoWinExaminerViewer* viewer = new SoWinExaminerViewer(window);
	SoSeparator* root = new SoSeparator();
	root->ref();

	//char meshFile[300] = "C:/Users/kca/Desktop/courses/789digital/789_hw1_kadircenk/input_meshes/man0.off";
	char meshFile[300];
	printf_s("Enter mesh file path: ");
	scanf_s("%s", &meshFile, 300);
	Mesh* mesh = new Mesh(meshFile);

	int qNo;
	printf_s("Enter question number (1, 2, 3): ");
	scanf_s("%d", &qNo);

	printf_s("Geo. dist. data structure (0: array, 1: min-heap, 2: fibo-heap): ");
	scanf_s("%d", &dataStructure);

	printf_s("Sphere visual scale (1: small, 2: big): ");
	scanf_s("%f", &sphereScale);
	if (sphereScale == 1)
		sphereScale = 0.02;
	else
		sphereScale = 4;

	if (qNo == 1)
		solveQ1(root, mesh);
	else if (qNo == 2)
		solveQ2(root, mesh);
	else if (qNo == 3)
		solveQ3(root, mesh);
	else
		exit(0);

	viewer->setSize(SbVec2s(1024, 768));
	viewer->setSceneGraph(root);
	viewer->show();
	SoWin::show(window);
	SoWin::mainLoop();
	delete viewer;
	root->unref();
	return 0;
}


//fibo-heap "handle" implementation backup
//else if (dataStructure == 2) //fibo heap
//	{
//	boost::heap::fibonacci_heap<heap_data> fiboHeap;
//
//	//init dijkstra costs, 0 for vertex param, nearly infinity for other N-1 verts
//	for (size_t i = 0; i < mesh->verts.size(); i++)
//	{
//		mesh->verts[i]->curProcessed = false;
//		mesh->verts[i]->curGeoDist = 99999.0f;
//		mesh->verts[i]->curGeoParent = NULL;
//
//
//		//	MinHeapVertex tmp(mesh->verts[i]);
//		//heap_data a(mesh->verts[i]->curGeoDist);
//		//boost::heap::fibonacci_heap<heap_data>::handle_type handle = fiboHeap.push(a);
//		//heap_data f(2);
//
//		//boost::heap::fibonacci_heap<heap_data>::handle_type handle = heap.push(f);
//		//(*handle).handle = handle; // store handle in node
//
//
//		//fibonacci_heap<heap_data> heap;
//		//heap_data f(2);
//
//		//fibonacci_heap<heap_data>::handle_type handle = heap.push(f);
//		//(*handle).handle = handle; // store handle in node
//
//	}
//
//	vertex->curGeoDist = 0.0f;
//	heap_data a(vertex);
//
//	boost::heap::fibonacci_heap<heap_data>::handle_type handle = fiboHeap.push(a);
//	(*handle).handle = handle;
//
//	//fiboHeap.update(fiboHeapHandlesVec[vertex->idx]);
//
//
//	while (!fiboHeap.empty()) {
//		Vertex* minVert = fiboHeap.top().v; //extract min from min-heap
//		fiboHeap.pop();
//
//		//islenmisse devam et
//		if (minVert->curProcessed)
//			continue;
//
//		//simdi isliyorum
//		minVert->curProcessed = true;
//
//		// relax for each adj vert
//		relaxDijkstra(mesh, minVert);
//
//		for (size_t i = 0; i < minVert->vertList.size(); i++)
//		{
//			//islenmemisse ekle.
//			if (!(mesh->verts[minVert->vertList[i]]->curProcessed)) {
//				heap_data a(mesh->verts[minVert->vertList[i]]);
//
//				boost::heap::fibonacci_heap<heap_data>::handle_type handle = fiboHeap.push(a);
//				(*handle).handle = handle;
//			}
//		}
//	}
//	}