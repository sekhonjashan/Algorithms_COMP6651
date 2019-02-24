#include <iostream>
#include <fstream>
#include <vector>
using namespace std;

struct Node{
    int from;
    int to;
    int amount;
};
int totalPathWeight;
int findMatchedNodeWeight(int u , int v, vector<Node> nodeCollection){
    for(int i =0;i< nodeCollection.size();i++){
        if(nodeCollection[i].from == u && nodeCollection[i].to == v){
            return nodeCollection[i].amount;
        }else if(nodeCollection[i].from == v && nodeCollection[i].to == u){
            return nodeCollection[i].amount;
        }
    }
}
void HamiltonTravellingSalesMan(vector<Node> nodeCollection, vector<int> *data, int v, int _visitedList[], vector<int> _graphColl, int vcount, vector<int> weight)
{
	int len = _graphColl.size();
	if (vcount == len)
	{
	    int min_weight = 0;
        for(int w = 0 ;w < weight.size();w++){
            min_weight = min_weight + weight[w];
        }
        
		for(int j =0; j < len ; j++){
		  //  cout << _graphColl[j] << " ";
		    
                if((j+1) == len){
                    // cout << findMatchedNodeWeight(_graphColl[0], _graphColl[j]);
                     min_weight = min_weight + findMatchedNodeWeight(_graphColl[0], _graphColl[j], nodeCollection);
                }
        }
        if(totalPathWeight ==0 || totalPathWeight > min_weight){
            totalPathWeight = min_weight;
        }
		return;
	}

	for(int j=0; j< data[v].size();j++){
	    int val = data[v][j];
	    if (_visitedList[val] == 0)
		{
		    weight.push_back(findMatchedNodeWeight(v,val, nodeCollection));
			_graphColl.push_back(val);
			_visitedList[val] = 1;
			HamiltonTravellingSalesMan(nodeCollection, data, val, _visitedList, _graphColl, vcount , weight);
			weight.pop_back();
			_graphColl.pop_back();
			_visitedList[val] = 0;
		}
	}
}
int main(int argc, char* argv[])
{
	if(argc != 3)
  {
    cout << "Error: Incorrect number of arguments!!!" << endl;
    return -1;
  }

	ifstream infile(argv[1]);
	ofstream out_put;
	out_put.open(argv[2]);
  if(!infile)
  {
    cout << "Error: Cannot open input file!!!" << endl;
    return -1;
  }

  if(!out_put)
  {
    cout << "Error: Cannot open output file (Permission issue?) !!!" << endl;
    return -1;
  }
	int dataSets;
	infile>>dataSets;
	for(int d=0;d<dataSets;d++){
		totalPathWeight = 0;
		vector<Node> nodeCollection;
		vector<int> *collectionList;
		int verticesCount,edgesCount;
		infile>>verticesCount>>edgesCount;
		collectionList = new vector<int>[verticesCount];
		int *_visitedList = new int[verticesCount];
		for (int i = 0; i < verticesCount; i++){
			_visitedList[i] = 0;
		}
		int index = 0;
		vector<int> _graphColl;
		_graphColl.push_back(index);
		_visitedList[index] = 1;
		vector<int> weight;
		for(int e=0; e< edgesCount;e++){
			int _from, _to, _weight;
			infile>>_from>>_to>>_weight;
			Node _node {_from,_to,_weight};
			nodeCollection.push_back(_node);
			collectionList[_from].push_back(_to);
			collectionList[_to].push_back(_from);
		}
		HamiltonTravellingSalesMan(nodeCollection, collectionList, index, _visitedList, _graphColl, verticesCount , weight);
		out_put << totalPathWeight << endl;
	}
	infile.close();
    return 0;
}

