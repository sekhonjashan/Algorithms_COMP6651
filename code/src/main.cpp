#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;
Mat energy_scale_create(Mat &scale_img, bool isVertical){
	Mat opacity_img, gr_img,xgr,ygr, xag, yag, gd, e_img_scale ,en_map;
	GaussianBlur(scale_img, opacity_img, Size(3,3), 0, 0, BORDER_DEFAULT);
	cvtColor(opacity_img, gr_img, CV_BGR2GRAY);
	Sobel(gr_img, xgr, CV_16S, 1, 0, 3, 1, 0, BORDER_DEFAULT);
    Sobel(gr_img, ygr, CV_16S, 0, 1, 3, 1, 0, BORDER_DEFAULT);
	convertScaleAbs(xgr, xag);
    convertScaleAbs(ygr, yag);
	addWeighted(xag, 0.5, yag, 0.5, 0, gd);
	gd.convertTo(e_img_scale, CV_64F, 1.0/255.0);
	int rows_len = e_img_scale.rows, cols_len = e_img_scale.cols;
	double init = double(0);
	en_map = Mat(rows_len ,cols_len, CV_64F, init);
	if(isVertical){
		for(int x = 0; x< cols_len; ++x){
			 en_map.at<double>(0, x) = e_img_scale.at<double>(0, x);
		}
		for(int x = 1; x < rows_len;++x){
			for(int y=0 ; y< cols_len; ++y){
				double points;
				 if (y == 0) {
                    points =  min(en_map.at<double>(x-1, 0), en_map.at<double>(x-1, 1));
                } else if (y == cols_len-1) {
                    points =  min(en_map.at<double>(x-1, cols_len-1), en_map.at<double>(x-1, cols_len-2));
                } else {
                    points = min(en_map.at<double>(x-1, y-1), min(en_map.at<double>(x-1, y), en_map.at<double>(x-1, y+1)));
                }
				en_map.at<double>(x, y) = e_img_scale.at<double>(x, y) + points;
			}
		}
	}else{
		for (int x = 0; x < rows_len; ++x) {
            en_map.at<double>(x, 0) = e_img_scale.at<double>(x, 0);
        }
		for(int y = 1; y < cols_len;++y){
			for(int x=0 ; x< rows_len; ++x){
				double points;
				if (x == 0) {
                    points = min(en_map.at<double>(0, y-1), en_map.at<double>(1, y-1));
                } else if (x == rows_len-1) {
                    points =  min(en_map.at<double>(rows_len-1, y-1), en_map.at<double>(rows_len-2, y-1));
                } else {
                    points = min(en_map.at<double>(x-1, y-1), min(en_map.at<double>(x, y-1), en_map.at<double>(x+1, y-1)));
                }
				en_map.at<double>(x, y) = e_img_scale.at<double>(x, y) + points;
			}
		}
	}
	return en_map;
}
vector<int> minimum_path(Mat &cm_emap, bool isVertical) {
    vector<int> seam_path;
    double val_min ,val_max;
    Point ptx_min, ptx_max;
    int x_col = cm_emap.rows,y_row = cm_emap.cols,index = 0,optim_index;
	minMaxLoc((isVertical ? cm_emap.row(x_col - 1) : cm_emap.col(y_row - 1)), &val_min, &val_max, &ptx_min, &ptx_max);
	seam_path.resize(isVertical ? x_col : y_row);
    optim_index = isVertical ? ptx_min.x : ptx_min.y;;
    if (isVertical) {
        seam_path[x_col - 1] = optim_index;
        for (int x = x_col - 2; x >= 0; x--) {
            if (min(cm_emap.at<double>(x, max(optim_index - 1, 0)),cm_emap.at<double>(x, optim_index)) > cm_emap.at<double>(x, min(optim_index + 1, y_row - 1))) {
                index = 1;
            }
            else if (min(cm_emap.at<double>(x, max(optim_index - 1, 0)),cm_emap.at<double>(x, min(optim_index + 1, y_row - 1))) > cm_emap.at<double>(x, optim_index)) {
                index = 0;
            }
            else if (min(cm_emap.at<double>(x, optim_index), cm_emap.at<double>(x, min(optim_index + 1, y_row - 1))) > cm_emap.at<double>(x, max(optim_index - 1, 0))) {
                index = -1;
            }
            optim_index += index;
            optim_index = min(max(optim_index, 0), y_row - 1);
            seam_path[x] = optim_index;
        }
    }
    else {
        seam_path[y_row - 1] = optim_index;
        for (int x = y_row - 2; x >= 0; x--) {
            if (min(cm_emap.at<double>(max(optim_index - 1, 0), x),cm_emap.at<double>(optim_index, x)) > cm_emap.at<double>(min(optim_index + 1, x_col - 1), x)) {
                index = 1;
            }
            else if (min(cm_emap.at<double>(max(optim_index - 1, 0), x),cm_emap.at<double>(min(optim_index + 1, x_col - 1), x)) > cm_emap.at<double>(optim_index, x)) {
                index = 0;
            }
            else if (min(cm_emap.at<double>(optim_index, x), cm_emap.at<double>(min(optim_index + 1, x_col - 1), x)) > cm_emap.at<double>(max(optim_index - 1, 0), x)) {
                index = -1;
            }
            optim_index += index;
            optim_index = min(max(optim_index, 0), x_col - 1); 
            seam_path[x] = optim_index;
        }
    }
    return seam_path;
}
void remove_seams(Mat &d, Mat &u, Mat &g, bool isVertical){
	if(isVertical){
		hconcat(d, u, g);
	}else{
		vconcat(d, u, g);
	}
}
void remove_seam_trivial(int x_len, int y_len, Mat &ip_img, vector<int> scale_path, bool isVertical){
	Mat alt(1, 1, CV_8UC3, Vec3b(0, 0, 0));
	for(int v =0 ; v< x_len;++v){
			Mat gen_max, down, up;
			if(isVertical){
				down = ip_img.rowRange(v, v + 1).colRange(0, scale_path[v]);
				up = ip_img.rowRange(v, v + 1).colRange(scale_path[v] + 1, y_len);
			}else{
				down = ip_img.colRange(v, v + 1).rowRange(0, scale_path[v]);
				up = ip_img.colRange(v, v + 1).rowRange(scale_path[v] + 1, y_len);
			}
            if (!down.empty() && !up.empty()) {
				if(isVertical){
					remove_seams(down, up, gen_max, true);
					remove_seams(gen_max, alt, gen_max, true);
				}else{
					remove_seams(down, up, gen_max, false);
					remove_seams(gen_max, alt, gen_max, false);
				}
            }
            else {
				if (up.empty()) {
					if(isVertical){
						remove_seams(down, alt, gen_max, true);
					}else{
						remove_seams(down, alt, gen_max, false);
					}
                }else if (down.empty()) {
					if(isVertical){
						remove_seams(up, alt, gen_max, true);
					}else{
						remove_seams(up, alt, gen_max, false);
					}
                }
            }
			if(isVertical){
				gen_max.copyTo(ip_img.row(v));
			}else{
				gen_max.copyTo(ip_img.col(v));
			}
		}
}

void reduce_seam_trivial(vector<int> scale_path, Mat &ip_img, bool isVertical){
	int row_len = ip_img.rows, col_len = ip_img.cols;
	if(isVertical){
		remove_seam_trivial(row_len,col_len,ip_img,scale_path,isVertical);
		ip_img = ip_img.colRange(0, col_len - 1);
	}else{
		remove_seam_trivial(col_len,row_len,ip_img,scale_path,isVertical);
		ip_img = ip_img.rowRange(0, row_len - 1);
	}
}
Mat seam_carving(Mat &iip_image, Mat &out_image, int set_width, int set_height){
	int iteration_x = (iip_image.cols - set_width), iteration_y = (iip_image.rows - set_height); 
	int width =  0, height = 0;
	while(width != iteration_x || height != iteration_y){
		if(width < iteration_x){
			width += 1;
			Mat carvy_img = energy_scale_create(iip_image , true);
			reduce_seam_trivial(minimum_path(carvy_img, true), iip_image, true);
		}
		if(height < iteration_y){
			height += 1;
			Mat carvy_img = energy_scale_create(iip_image , false);
			reduce_seam_trivial(minimum_path(carvy_img, false), iip_image, false);
		}
	}
	return iip_image;
}
int main( int argc, char** argv )
{
    if(argc!=5){
        cout<<"Usage: ../sc input_image new_width new_height output_image"<<endl;
        return -1;
    }
    // Load the input image
    // the image should be a 3 channel image by default but we will double check that in teh seam_carving
    Mat in_image;
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
    
    // get the new dimensions from the argument list
    int new_width = atoi(argv[2]);
    int new_height = atoi(argv[3]);
    
    // the output image
    Mat out_image = in_image.clone();
    int rows = in_image.rows , cols = in_image.cols;
    if(new_width>cols){
        cout<<"Invalid request!!! new_width has to be smaller than the current size!"<<endl;
        return -1;
    }
    if(new_height>rows){
        cout<<"Invalid request!!! ne_height has to be smaller than the current size!"<<endl;
        return -1;
    }
    if(new_width<=0){
        cout<<"Invalid request!!! new_width has to be positive!"<<endl;
        return -1;

    }
	 if(new_height<=0){
        cout<<"Invalid request!!! new_height has to be positive!"<<endl;
        return -1;
        
    }
    // write it on disk
    imwrite( argv[4], seam_carving(in_image, out_image, new_width, new_height));
    
    // also display them both
    
    namedWindow( "Original image", WINDOW_AUTOSIZE );
    namedWindow( "Seam Carved Image", WINDOW_AUTOSIZE );
    imshow("Seam Carved Image", in_image );
    imshow( "Original image", out_image );
    waitKey(0);
    return 0;
}
