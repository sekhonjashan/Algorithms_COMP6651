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

struct Object {
  int $vertices, $path, w, $x, $y;
};

vector <Object> * adjacentVector;
int totalNodes;
int * lookup;

bool Search(int src, int snk)
{
	for (int b = 0; b < totalNodes; b++) {
		lookup[b] = !(b == src) ? -1 : 0;
	}
	queue<int> que;
    que.push(src);
    vector<Object> ::iterator loop;
    while (!que.empty())
    {
        int $front = que.front();
        que.pop();
        for (loop=adjacentVector[$front].begin(); loop!=adjacentVector[$front].end(); ++loop)
        {
			if(lookup[snk] > 0){
				return true;
			}
			Object &obj = *loop;
			if (lookup[obj.$vertices] < 0)
			{
				if(obj.$path < obj.w){
					lookup[obj.$vertices] = lookup[$front] + 1;
					que.push(obj.$vertices);
				}
			}
        }
    }
	return (lookup[snk] > 0);
}

int depthFlow(int $verti, int path, int snk, int adj[])
{
	for (; adj[$verti] < adjacentVector[$verti].size(); adj[$verti]++)
	{
		if ($verti == snk){
			return path;
		}
		Object &obj = adjacentVector[$verti][adj[$verti]];
		if (obj.$path < obj.w && (lookup[obj.$vertices] == lookup[$verti] + 1))
		{
			int treePath = depthFlow(obj.$vertices, min(path, obj.w - obj.$path), snk, adj);
			if (treePath > 0)
			{
				obj.$path = obj.$path + treePath;
				int len = adjacentVector[obj.$vertices].size();
				adjacentVector[obj.$vertices][len].$path -= treePath;
				return treePath;
			}
		}
	}
	return 0;
}

void colorImage(Mat &out_image , int color, int x , int y){
	Vec3b $pixel;
	$pixel[0] = color;
	$pixel[1] = color;
	$pixel[2] = color;
	out_image.at<Vec3b>(x, y) = $pixel;
}

void graphCut(int src, int snk, Mat &out_image, Mat &$bgimg)
{
	  while (Search(src, snk)) {
		int *adj = new int[totalNodes + 1];
		while (int flow = depthFlow(src, INT_MAX, snk, adj))
		  flow +=flow; 
	  }
	int rows = $bgimg.rows, cols = $bgimg.cols;
	for (int row = 0; row < rows; row++)
	{
		for (int _col = 0; _col < cols; _col++)
		{
			colorImage(out_image,0,row, _col);
		}
	}
	
	vector<Object>::iterator loop;
	for (int b = 0; b < totalNodes; b++) {
		lookup[b] = !(b == src) ? -1 : 0;
	}
	queue< int > que;
	que.push(src);
	while (!que.empty())
	{
		int $front = que.front();
		que.pop();
		for (loop = adjacentVector[$front].begin(); loop != adjacentVector[$front].end(); loop++)
		{
			Object &obj = *loop;
			if (lookup[obj.$vertices] < 0 && obj.$path < obj.w)
			{
				colorImage(out_image, 255 , obj.$x, obj.$y);
				lookup[obj.$vertices] = lookup[$front] + 1;
				que.push(obj.$vertices);
			}
		}
	}
}

void pushDataEdge(int x, int y, int vertices_x, int vertices, double weight){
	Object frObj;
	frObj.$vertices = vertices;
	frObj.$path = 0;
	frObj.w =weight;
	frObj.$x = x;
	frObj.$y = y ;
	adjacentVector[vertices_x].push_back(frObj);
	Object sedcondObj;
	sedcondObj.$vertices = vertices_x;
	sedcondObj.$path = 0;
	sedcondObj.w = 0;
	sedcondObj.$x = x;
	sedcondObj.$y = y + 1 ;
    adjacentVector[vertices].push_back(sedcondObj);
}

double getGussianProbability(double GP, double $intensity, double $avg, double $mean, double $energy){
	double $probability;
	 if ($avg == 0) {
			double $powsqt = pow(($intensity - $mean), 2);
			$probability =  GP * exp(-0.5d * $powsqt);
		} else {
			double $powsqt = pow(($energy - $mean), 2);
			$probability = (GP / sqrt($avg)) * exp(-0.5d * ($powsqt / $avg));
		}
	return $probability;
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
	bg_image = in_image.clone();
    ifstream f(argv[2]);
    if(!f){
        cout<<"Could not load initial mask file!!!"<<endl;
        return -1;
    }
    
    int width = in_image.cols;
    int height = in_image.rows;
    
    int n;
    f>>n;
    double energyCol[height][width];
	for(int i=0;i<height;i++){
		for(int j=0;j<width;j++){
			Vec3b $int = bg_image.at<Vec3b>(i, j);
			int totalInten = ($int[0] + $int[1] + $int[2]);
			energyCol[i][j] = (totalInten/3);
		}
	}
    // get the initial pixels
	int input_pixels[n][3];
	int $flen =0, $blen = 0;
	double $fnMeanEn =0,$fnMeanAdj=0,$bnMeanEn =0,$bnMeanAdj=0, enVar = 0, bnVar = 0,fSigma,bSigma;
    for(int i=0;i<n;++i){
        int x, y, t;
        f>>x>>y>>t;
        
        if(x<0 || x>=width || y<0 || y>=height){
            cout<<"Invalid pixel mask!"<<endl;
            return -1;
        }
		if(t ==1){
			$flen = $flen + 1;
			Vec3b $int = bg_image.at<Vec3b>(y, x);
			int totalInten = ($int[0] + $int[1] + $int[2]);
			$fnMeanEn  = $fnMeanEn + energyCol[y][x];
			//cout<<energyCol[y][x]<<endl;
			$fnMeanAdj = $fnMeanAdj + (totalInten/3);
		}else{
			$blen = $blen + 1;
			Vec3b $int = bg_image.at<Vec3b>(y, x);
			int totalInten = ($int[0] + $int[1] + $int[2]);
			$bnMeanEn  = $bnMeanEn + energyCol[y][x];
			$bnMeanAdj = $bnMeanAdj + ( totalInten/3);
		}
        input_pixels[i][1] = x;
		input_pixels[i][0] = y;
		input_pixels[i][2] = t;
    }
    cout<<"..."<<endl;
	  for (int i = 0; i < n; ++i) {
		int $x = input_pixels[i][0], $y = input_pixels[i][1];
		if (input_pixels[i][2] == 1) {
		  double $entity = ($fnMeanEn / $flen);
		  enVar += pow(energyCol[$x][$y] - $entity, 2);
		} else {
		  double $entity = ($bnMeanEn / $blen);
		  bnVar += pow(energyCol[$x][$y] - $entity, 2);
		}
	  }
	//vector<map<int,int> > adjacentVector;
	fSigma = enVar/($flen-1);
	bSigma = bnVar/($blen-1);
	bool $flag;
	totalNodes = (width * height) + 2;
	adjacentVector = new vector <Object>[totalNodes];
	lookup = new int[totalNodes];
	int rows = bg_image.rows , cols = bg_image.cols, $count = 0;
	static const double GP = 0.399;
	int segSize = (rows*cols);
	
	for(int x=0;x<rows;++x){
		for(int y=0; y<cols;++y){
			$flag = false;
			for (int ip = 0; ip < n; ++ip) {
				int _x = input_pixels[ip][0], _y = input_pixels[ip][1];
				if (x == _x && y == _y) {
					if(input_pixels[ip][2] == 1){
						pushDataEdge(x, y, segSize, $count, 2);
						pushDataEdge(x, y, $count, segSize +1, 1);
					}else{
						pushDataEdge(x, y, segSize, $count, 1);
						pushDataEdge(x, y, $count, segSize+1, 2);
					}
					$flag = true;
					break;
				}
			  }
			  double $fAverage, $bAverage;
			  if (!$flag) {
				Vec3b $int = bg_image.at<Vec3b>(x, y);
				int totalInten = ($int[0] + $int[1] + $int[2])/3;
				$fAverage = getGussianProbability(GP, totalInten,fSigma, ($fnMeanEn / $flen), energyCol[x][y]);
				$bAverage = getGussianProbability(GP, totalInten, bSigma, ($bnMeanEn / $blen), energyCol[x][y]);
				$fAverage /= ($fAverage + $bAverage);
				$bAverage /=($fAverage + $bAverage);
				if($fAverage < $bAverage){
					pushDataEdge(x, y, segSize, $count, 1);
					pushDataEdge(x, y, $count, segSize+1, 2);
				}else{
					pushDataEdge(x, y, segSize, $count, 2);
					pushDataEdge(x, y, $count, segSize +1, 1);
				}
			  }
			  int fnNode = rows-1, bnNode = cols-1;
			  if ((x < fnNode) || (y < bnNode)) {
				  Vec3b $next, $currentInten;
				  if(x < fnNode){
					$next = (x != fnNode) ? bg_image.at<Vec3b> (x + 1, y) : bg_image.at<Vec3b> (0, y);
				  }else if(y < bnNode){
					$next = (y != bnNode) ? bg_image.at<Vec3b> (x, y + 1) : bg_image.at<Vec3b> (x, 0);
				  }
				  $currentInten = bg_image.at<Vec3b> (x, y);
				  double $weight = sqrt(pow($next[0] - $currentInten[0], 2) + pow($next[1] - $currentInten[1], 2) + pow($next[2] - $currentInten[2], 2));
				  $weight = ($weight <= 0) ? 2 : 1;
				  int $findVerti = (fnNode > x) ? ($count + cols) : ($count + 1);
				  pushDataEdge(x, y, $count, $findVerti, $weight);
			  }
			  $count++;
		}
	}
	graphCut(segSize,(segSize+1), out_image, bg_image);
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
