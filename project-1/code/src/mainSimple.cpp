#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <map>
#include <climits>
#include <limits.h>
#include <queue>
#include <cstring>

using namespace cv;
using namespace std;

bool checkIsExist(int x, int y, vector<map<int,int> > &adj);
int BFS(int source, int sink, vector<int> &edges, vector<map<int,int> > &adj);
int getLine(int x, int y, int max_flow, bool isVisited, vector<map<int,int> > &adj , bool isUpdate);

bool checkIsExist(int x, int y, vector<map<int,int> > &adj){
    map<int,int> :: iterator next;
    for (next = adj.at(x).begin(); next != adj.at(x).end(); ++next) {
        if (next->first == y) {
            if(next->second != 0){
                return true;
            }
        }
    }
    return false;
}

int BFS(int source, int sink, vector<int> &edges, vector<map<int,int> > &adj)
{
    int lookup[adj.size()];
    memset(lookup, 0, sizeof(lookup));
    lookup[source] = 1;
    edges[source] = 0;
    queue <int> _que;
    _que.push(source);
    while (!_que.empty())
    {
        int $front = _que.front();
        _que.pop();
        map<int,int>::iterator _ite;
        for (_ite=adj.at($front).begin(); _ite!=adj.at($front).end(); ++_ite)
        {
            if(sink == $front){
                return true;
            }
            if (checkIsExist($front, _ite->first, adj) && lookup[_ite->first] == 0)
            {
                edges[_ite->first] = $front;
                _que.push(_ite->first);
                lookup[_ite->first] = 1;
            }
        }
    }
    return lookup[sink] == 1;
}

int getLine(int x, int y, int max_flow, bool isVisited, vector<map<int,int> > &adj , bool isUpdate){
    map<int,int> :: iterator next;
    for (next = adj.at(x).begin(); next != adj.at(x).end(); ++next) {
        if(next->first == y){
            if(isUpdate){
                return next->second = (isVisited) ? (next->second) - max_flow : (next->second) + max_flow;
            }else{
                return next->second;
            }
        }
    }
}
void graphCut(int source, int sink, Mat &out_image, vector<map<int,int> > &adjVec)
{
    vector<map<int,int> > adj = adjVec;
    vector<int> edges(adj.size());
    int vertices, weight, max_flow = INT_MAX/2;
    while (BFS(source, sink, edges, adj))
    {
        for (weight=sink; weight!=source; weight=edges[weight])
        {
            vertices = edges[weight];
            max_flow = min(max_flow, getLine(vertices, weight, max_flow, true, adj, false));
        }

        for (weight=sink; weight != source; weight=edges[weight])
        {
            vertices = edges[weight];
            getLine(vertices, weight, max_flow, true, adj, true);
            getLine(weight, vertices, max_flow, false, adj, true);
        }
    }
    int *lookup= new int[adj.size()];
    memset(lookup, false, adj.size());
    lookup[source] = 1;
    queue <int> _que;
    _que.push(source);
    while(!_que.empty()){
        int $front =_que.front();
        _que.pop();
        map<int,int>::iterator _ite;
        for (_ite=adj.at($front).begin(); _ite!=adj.at($front).end(); ++_ite)
        {
            if (lookup[_ite->first]==0 && checkIsExist($front, _ite->first, adj))
            {
                _que.push(_ite->first);
                lookup[_ite->first] = 1;
            }
        }
    }
    for(int c=0;c<adj.size()-2;++c){
        Vec3b intensity;
        if(lookup[c] == 1){
            intensity[0] = intensity[1] = intensity[2] = 255;
        }else{
            intensity[0] = intensity[1] = intensity[2] = 0;
        }
        out_image.at<Vec3b>(c/out_image.cols,c%out_image.cols) = intensity;
    }
}
bool checkMatrix(Mat &bg_img, int x ,int y, int _x, int _y){
    return (bg_img.at<uchar>(x,y) == bg_img.at<uchar>(_x,_y));
}
int main( int argc, char** argv )
{
    if(argc!=4){
        cout<<"Usage: ../seg input_image initialization_file output_mask"<<endl;
        return -1;
    }
    
    // Load the input image
    // the image should be a 3 channel image by default but we will double check that in teh seam_carving
    Mat in_image, bg_image;
    in_image = imread(argv[1]/*, CV_LOAD_IMAGE_COLOR*/);
   
    if(!in_image.data)
    {
        cout<<"Could not load input image!!!"<<endl;
        return -1;
    }

    if(in_image.channels()!=3){
        cout<<"Image does not have 3 channels!!! "<<in_image.depth()<<endl;
        return -1;
    }
    
    // the output image
    Mat out_image = in_image.clone();
    cvtColor(in_image,bg_image,CV_BGR2GRAY);
    ifstream f(argv[2]);
    if(!f){
        cout<<"Could not load initial mask file!!!"<<endl;
        return -1;
    }
    
    int width = in_image.cols;
    int height = in_image.rows;
    
    int n;
    f>>n;
    
    // get the initil pixels
    int input_pixels[n][3];
    for(int i=0;i<n;++i){
        int x, y, t;
        f>>x>>y>>t;
        
        if(x<0 || x>=width || y<0 || y>=height){
            cout<<"I valid pixel mask!"<<endl;
            return -1;
        }
        input_pixels[i][0] = x;
        input_pixels[i][1] = y;
        input_pixels[i][2] = t;
    }
    
    vector<map<int,int> > adjacentVector;
    int rows = bg_image.rows , cols = bg_image.cols , avg = INT_MAX/2, con_avg = 50;
    for(int x=0;x<bg_image.rows;++x){
        for(int y=0; y<bg_image.cols;++y){
            int _index = ((x*cols) + y);
            map<int,int> keyVal;
            adjacentVector.push_back(keyVal);
            if(x == 0){
                if(y== 0){
                    if(checkMatrix(bg_image,x, y, x,y+1)){
                        adjacentVector.at(_index)[(x*cols)+(y+1)] = avg;
                    }else{
                        adjacentVector.at(_index)[(x*cols)+(y+1)] = con_avg;
                    }
                    if(checkMatrix(bg_image,x, y, x+1,y)){
                        adjacentVector.at(_index)[((x+1)*cols)+y] = avg;
                    }else{
                        adjacentVector.at(_index)[((x+1)*cols)+y] = con_avg;
                    }
                }else if(cols-1 == y){
                    if(checkMatrix(bg_image,x, y, x,y-1)){
                        adjacentVector.at(_index)[(x*cols)+(y-1)] = avg;
                    }else{
                        adjacentVector.at(_index)[(x*cols)+(y-1)] = con_avg;
                    }
                    if(checkMatrix(bg_image,x, y, x+1,y)){
                        adjacentVector.at(_index)[((x+1)*cols)+y] = avg;
                    }else{
                        adjacentVector.at(_index)[((x+1)*cols)+y] = con_avg;
                    }
                }else{
                    if(checkMatrix(bg_image,x, y, x,y-1)){
                        adjacentVector.at(_index)[(x*cols)+(y-1)] = avg;
                    }else{
                        adjacentVector.at(_index)[(x*cols)+(y-1)] = con_avg;
                    }
                    
                    if(checkMatrix(bg_image,x, y, x,y+1)){
                        adjacentVector.at(_index)[(x*cols)+(y+1)] = avg;
                    }else{
                        adjacentVector.at(_index)[(x*cols)+(y+1)] = con_avg;
                    }
                    if(checkMatrix(bg_image,x, y, x+1,y)){
                        adjacentVector.at(_index)[((x+1)*cols)+y] = avg;
                    }else{
                        adjacentVector.at(_index)[((x+1)*cols)+y] = con_avg;
                    }
                }
                }else if(rows-1 == x){
                    if(y==0){
                        if(checkMatrix(bg_image,x, y, x-1,y)){
                            adjacentVector.at(_index)[((x-1)*cols)+y] = avg;
                        }else{
                            adjacentVector.at(_index)[((x-1)*cols)+y] = con_avg;
                        }
                        
                        if(checkMatrix(bg_image,x, y, x,y+1)){
                            adjacentVector.at(_index)[(x*cols)+(y+1)] = avg;
                        }else{
                            adjacentVector.at(_index)[(x*cols)+(y+1)] = con_avg;
                        }
                    }else if(cols-1 == y){
                        if(checkMatrix(bg_image,x, y, x-1,y)){
                            adjacentVector.at(_index)[((x-1)*cols)+y] = avg;
                        }else{
                            adjacentVector.at(_index)[((x-1)*cols)+y] = con_avg;
                        }
                        if(checkMatrix(bg_image,x, y, x,y-1)){
                            adjacentVector.at(_index)[(x*cols)+(y-1)] = avg;
                        }else{
                            adjacentVector.at(_index)[(x*cols)+(y-1)] = con_avg;
                        }
                    }else{
                        if(checkMatrix(bg_image,x, y, x-1,y)){
                            adjacentVector.at(_index)[((x-1)*cols)+y] = avg;
                        }else{
                            adjacentVector.at(_index)[((x-1)*cols)+y] = con_avg;
                        }
                        if(checkMatrix(bg_image,x, y, x,y+1)){
                            adjacentVector.at(_index)[(x*cols)+(y+1)] = avg;
                        }else{
                            adjacentVector.at(_index)[(x*cols)+(y+1)] = con_avg;
                        }
                        if(checkMatrix(bg_image,x, y, x,y-1)){
                            adjacentVector.at(_index)[(x*cols)+(y-1)] = avg;
                        }else{
                            adjacentVector.at(_index)[(x*cols)+(y-1)] = con_avg;
                        }
                    }
                }else{
                    if (y == 0) {
                        if(checkMatrix(bg_image,x, y, x-1,y)){
                            adjacentVector.at(_index)[((x-1)*cols)+y] = avg;
                        }else{
                            adjacentVector.at(_index)[((x-1)*cols)+y] = con_avg;
                        }
                        if(checkMatrix(bg_image,x, y, x,y+1)){
                            adjacentVector.at(_index)[(x*cols)+(y+1)] = avg;
                        }else{
                            adjacentVector.at(_index)[(x*cols)+(y+1)] = con_avg;
                        }
                        if(checkMatrix(bg_image,x, y, x+1,y)){
                            adjacentVector.at(_index)[((x+1)*cols)+y] = avg;
                        }else{
                            adjacentVector.at(_index)[((x+1)*cols)+y] = con_avg;
                        }
                    }else if(cols-1 == y){
                        if(checkMatrix(bg_image,x, y, x-1,y)){
                            adjacentVector.at(_index)[((x-1)*cols)+y] = avg;
                        }else{
                            adjacentVector.at(_index)[((x-1)*cols)+y] = con_avg;
                        }
                        if(checkMatrix(bg_image,x, y, x,y-1)){
                            adjacentVector.at(_index)[(x*cols)+(y-1)] = avg;
                        }else{
                            adjacentVector.at(_index)[(x*cols)+(y-1)] = con_avg;
                        }
                        if(checkMatrix(bg_image,x, y, x+1,y)){
                            adjacentVector.at(_index)[((x+1)*cols)+y] = avg;
                        }else{
                            adjacentVector.at(_index)[((x+1)*cols)+y] = con_avg;
                        }
                    }else{
                        if(checkMatrix(bg_image,x, y, x-1,y)){
                            adjacentVector.at(_index)[((x-1)*cols)+y] = avg;
                        }else{
                            adjacentVector.at(_index)[((x-1)*cols)+y] = con_avg;
                        }
                        if(checkMatrix(bg_image,x, y, x,y+1)){
                            adjacentVector.at(_index)[(x*cols)+(y+1)] = avg;
                        }else{
                            adjacentVector.at(_index)[(x*cols)+(y+1)] = con_avg;
                        }
                        if(checkMatrix(bg_image,x, y, x+1,y)){
                            adjacentVector.at(_index)[((x+1)*cols)+y] = avg;
                        }else{
                            adjacentVector.at(_index)[((x+1)*cols)+y] = con_avg;
                        }
                        if(checkMatrix(bg_image,x, y, x,y-1)){
                            adjacentVector.at(_index)[(x*cols)+(y-1)] = avg;
                        }else{
                            adjacentVector.at(_index)[(x*cols)+(y-1)] = con_avg;
                        }
                }
            }
        }
    }
    int segSize = (height*width);
    adjacentVector.resize(segSize+2);
    cout<<"..."<<endl;
    for(int o=0; o<n; ++o){
        int _x = input_pixels[o][0];
        int _y = input_pixels[o][1];
        int _t = input_pixels[o][2];
        int vlen;
        if(_t ==1){
            vlen = adjacentVector.size()-2;
        }else{
            vlen = adjacentVector.size()-1;
        }
        adjacentVector.at(vlen)[(_y*width)+_x] = avg;
        adjacentVector.at((_y*width)+_x)[vlen] = avg;
        
    }
    graphCut(segSize,(segSize+1), out_image, adjacentVector);
    // write it on disk
    imwrite( argv[3], out_image);
    cout<< "done" << endl;
    // also display them both
    
    namedWindow( "Original image", WINDOW_AUTOSIZE );
    namedWindow( "Show Marked Pixels", WINDOW_AUTOSIZE );
    imshow( "Original image", in_image );
    imshow( "Show Marked Pixels", out_image );
    waitKey(0);
    return 0;
}

