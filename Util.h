#pragma once

#include <stdlib.h>
#include<vector>

using namespace std;

class Util
{
public:
	//write NxN tumGeodesicDistances matrix to filePath
	static void writeMatrixToFile(vector<vector<float>> tumGeodesicDistances, const char* filePath) {
		FILE* matrisFp;
		fopen_s(&matrisFp, filePath, "w");
		if (!matrisFp)perror("fopen");

		//NxN
		for (size_t i = 0; i < tumGeodesicDistances.size(); i++)
		{
			for (size_t j = 0; j < tumGeodesicDistances[i].size(); j++)
			{
				fprintf(matrisFp, "%g ", tumGeodesicDistances[i][j]); // %g saves file size, cool.
			}
			fprintf(matrisFp, "\n");
		}
		fclose(matrisFp);
	}

	//to select OFF vs OBJ mesh loader, tutorial at https://stackoverflow.com/questions/5309471/getting-file-extension-in-c
	static const char* getFileExt(const char* filename) {
		const char* dot = strrchr(filename, '.');
		if (!dot || dot == filename) return "";
		return dot + 1;
	}

	//1-ring area calculation code block
	static float distanceTo(Vertex* v1, Vertex* v2) {
		return sqrt(pow(v1->coords[0] - v2->coords[0], 2.0) + pow(v1->coords[1] - v2->coords[1], 2.0) + pow(v1->coords[2] - v2->coords[2], 2.0));
	}

	static float getTriangleArea(Vertex* v1, Vertex* v2, Vertex* v3) {
		float a = distanceTo(v1, v2);
		float b = distanceTo(v2, v3);
		float c = distanceTo(v3, v1);
		float s = (a + b + c) / 2;
		return sqrt(s * (s - a) * (s - b) * (s - c));
	}

private:
	Util() {}
};
