#include "rrtTree.h"

rrtTree::rrtTree(){
    count = 0;
    root = NULL;
    ptrTable[0] = NULL;
}

rrtTree::rrtTree(point x_init, point x_goal, cv::Mat map, double map_origin_x, double map_origin_y, double res, int margin) {
    this->x_init = x_init;
    this->x_goal = x_goal;
    this->map_original = map.clone();
    this->map = addMargin(map, margin);
    this->map_origin_x = map_origin_x;
    this->map_origin_y = map_origin_y;
    this->res = res;

    count = 1;
    root = new node;
    ptrTable[0] = root;
    root->idx = 0;
    root->idx_parent = 0;
    root->location = x_init;
    root->rand = x_init;
}

rrtTree::~rrtTree(){
    for (int i = 1; i <= count; i++) {
        delete ptrTable[i];
    }
}

cv::Mat rrtTree::addMargin(cv::Mat map, int margin) {
    cv::Mat map_margin = map.clone();
    int xSize = map.cols;
    int ySize = map.rows;

    for (int i = 0; i < ySize; i++) {
        for (int j = 0; j < xSize; j++) {
            if (map.at<uchar>(i, j) < 125) {
                for (int k = i - margin; k <= i + margin; k++) {
                    for (int l = j - margin; l <= j + margin; l++) {
                        if (k >= 0 && l >= 0 && k < ySize && l < xSize) {
                            map_margin.at<uchar>(k, l) = 0;
                        }
                    }
                }
            }
        }
    }

    return map_margin;
}

void rrtTree::visualizeTree(){
    int thickness = 1;
    int lineType = 8;
    int idx_parent;
    double Res = 2;
    double radius = 6;
    cv::Point x1, x2;

    cv::Mat map_c;
    cv::Mat imgResult;
    cv::cvtColor(this->map_original, map_c, CV_GRAY2BGR);
    cv::resize(map_c, imgResult, cv::Size(), Res, Res);

    cv::circle(imgResult, cv::Point((int)(Res*(x_init.y/res + map_origin_y)), (int)(Res*(x_init.x/res + map_origin_x))), radius, cv::Scalar(0, 0, 255), CV_FILLED);
    cv::circle(imgResult, cv::Point((int)(Res*(x_goal.y/res + map_origin_y)), (int)(Res*(x_goal.x/res + map_origin_x))), radius, cv::Scalar(0, 0, 255), CV_FILLED);

    for(int i = 1; i < this->count; i++) {
        idx_parent = this->ptrTable[i]->idx_parent;
        x1 = cv::Point((int)(Res*(ptrTable[i]->location.y/res + map_origin_y)), (int)(Res*(ptrTable[i]->location.x/res + map_origin_x)));
        x2 = cv::Point((int)(Res*(ptrTable[idx_parent]->location.y/res + map_origin_y)), (int)(Res*(ptrTable[idx_parent]->location.x/res + map_origin_x)));
        cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
    }

    cv::namedWindow("Mapping");
    cv::Rect imgROI((int)Res*200,(int)Res*200,(int)Res*400,(int)Res*400);
    cv::imshow("Mapping", imgResult(imgROI));
    cv::waitKey(1);

}

void rrtTree::visualizeTree(std::vector<point> path){
    int thickness = 1;
    int lineType = 8;
    int idx_parent;
    double Res = 2;
    double radius = 6;
    cv::Point x1, x2;

    cv::Mat map_c;
    cv::Mat imgResult;
    cv::cvtColor(this->map_original, map_c, CV_GRAY2BGR);
    cv::resize(map_c, imgResult, cv::Size(), Res, Res);

    cv::circle(imgResult, cv::Point((int)(Res*(x_init.y/res + map_origin_y)), (int)(Res*(x_init.x/res + map_origin_x))), radius, cv::Scalar(0, 0, 255), CV_FILLED);
    cv::circle(imgResult, cv::Point((int)(Res*(x_goal.y/res + map_origin_y)), (int)(Res*(x_goal.x/res + map_origin_x))), radius, cv::Scalar(0, 0, 255), CV_FILLED);

    for(int i = 1; i < this->count; i++) {
        idx_parent = this->ptrTable[i]->idx_parent;
        x1 = cv::Point((int)(Res*(ptrTable[i]->location.y/res + map_origin_y)), (int)(Res*(ptrTable[i]->location.x/res + map_origin_x)));
        x2 = cv::Point((int)(Res*(ptrTable[idx_parent]->location.y/res + map_origin_y)), (int)(Res*(ptrTable[idx_parent]->location.x/res + map_origin_x)));
        cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
    }

    thickness = 3;
    for(int i = 1; i < path.size(); i++) {
        x1 = cv::Point((int)(Res*(path[i-1].y/res + map_origin_y)), (int)(Res*(path[i-1].x/res + map_origin_x)));
        x2 = cv::Point((int)(Res*(path[i].y/res + map_origin_y)), (int)(Res*(path[i].x/res + map_origin_x)));
        cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
    }

    cv::namedWindow("Mapping");
    cv::Rect imgROI((int)Res*200,(int)Res*200,(int)Res*400,(int)Res*400);
    cv::imshow("Mapping", imgResult(imgROI));
    cv::waitKey(1);

}

void rrtTree::addVertex(point x_new, point x_rand, int idx_near) {    
    //Loop through ptrTable to find where the first empty space is

    node *new_node = new node;
    new_node->idx = count;
    new_node->rand = x_rand;
    new_node->location = x_new;
    new_node->idx_parent = idx_near;
    ptrTable[count] = new_node;
    count++;
}


int rrtTree::generateRRT(double x_max, double x_min, double y_max, double y_min, int K, double MaxStep) {
    int k=1;
    point X_rand,X_new;
    int X_near_idx;
    while (k<=K)
    {
        X_rand=rrtTree::randomState(x_max,x_min,y_max,y_min);
        if (k%5 == 0){
            X_rand = x_goal;
            k++;
        } 
        X_near_idx=rrtTree::nearestNeighbor(X_rand);
        if (isCollision(X_rand, ptrTable[X_near_idx]->location)){
            continue;
        }
        X_new=rrtTree::newState(X_near_idx,X_rand,MaxStep);
        addVertex(X_new,X_rand,X_near_idx);
        k++;
    }
    return 0;
}


point rrtTree::randomState(double x_max, double x_min, double y_max, double y_min) {
    point result;
    result.x= rand()/(double)RAND_MAX*(x_max-x_min) + x_min;
    result.y= rand()/(double)RAND_MAX*(y_max-y_min) + y_min;
    return result;
}


point rrtTree::newState(int idx_near, point x_rand, double MaxStep) {
    point x_new;
    point x_near;

    //Find the coordinate of x_near in the array
    x_near=ptrTable[idx_near]->location;

    //Create the equation of the line between x_rand and x_near
    int i;
    double x;
    double y;
    double length;
    for (i = 0; i < 1000; i++){
        x = x_near.x + (x_rand.x-x_near.x)*i/999; 
        y = x_near.y + (x_rand.y-x_near.y)*i/999;
        length = std::sqrt(((x-x_near.x)*(x-x_near.x)) + (y-x_near.y)*(y-x_near.y));
        if (length > MaxStep){
            break;
        }
        
    }    
    x_new.x = x;
    x_new.y = y;
    return x_new;
}


int rrtTree::nearestNeighbor(point x_rand) {
    point x_current;
    double vector_length;
    int result=0;
    double min_length=1000000000;
    int i;
    for (i=0; i< count;i++)
    {
        //x_current the point correponded to the analysed node
        x_current.x=ptrTable[i]->location.x;
        x_current.y=ptrTable[i]->location.y;
        //Calculate the distance between the analysed point and the x_rand point
        vector_length=sqrt((x_rand.x-x_current.x)*(x_rand.x-x_current.x)+(x_rand.y-x_current.y)*(x_rand.y-x_current.y));
        if(vector_length<min_length)
        {
            min_length=vector_length;
            result=ptrTable[i]->idx;  //should be equal to i
        }
    }
    return result;
}

bool rrtTree::isCollision(point x1, point x2) {
    //Looping through the points one at a time to and check if they are occupied, remember to round the numbers, im guessing they are int inputs
    int i;
    for (i = 0; i < 100; i++){
        double x = x1.x + (x2.x-x1.x)*i/99; 
        double y = x1.y + (x2.y-x1.y)*i/99;
        if (map.at<uchar>(round(x/res + this->map_origin_x), round(y/res + this->map_origin_y)) == 0){ //There was a obstruction
            return true;
        }
    }    
    return false;
}


std::vector<point> rrtTree::backtracking(){
    int goal_index = nearestNeighbor(x_goal);
    std::vector<point> result;
    int i=goal_index;
    while (i !=0)
    {
        result.push_back(ptrTable[i]->location);    
        i=ptrTable[i]->idx_parent;
    }
    visualizeTree(result);
    return result;
}

double rrtTree::lineCost(node* n1, node* n2){
    double lCost=sqrt((n1->location.x-n2->location.x)*
        (n1->location.x-n2->location.x)+(n1->location.y-n2->location.y)*
        (n1->location.y-n2->location.y));
    return lCost;
}

double rrtTree::treeCost(node* n){
    double totalCost = 0;
    node* parent;
    while (n != root){
        parent = ptrTable[n->idx_parent];
        double costs = lineCost(parent, n);
        totalCost += costs;
        n = parent;
    }
    return totalCost;
}

 void rrtTree::treeNear(node* x, double radius){
    //Clear the vector
    Xnear.clear();
    double vector_length;
    point x_current;
    int i;
    for (i = 0; i < count; i++)
    {
        //x_current the point correponded to the analysed node
        x_current.x=ptrTable[i]->location.x;
        x_current.y=ptrTable[i]->location.y;
        //Calculate the distance between the analysed point and the x_rand point
        vector_length = sqrt((x->location.x-x_current.x)*(x->location.x-x_current.x)+(x->location.y-x_current.y)*(x->location.y-x_current.y));
        if(vector_length <= radius)
        {
            Xnear.push_back(ptrTable[i]);
        }
    }
}




int rrtTree::generateRRTst(double x_max, double x_min_d, double y_max, double y_min, int K, double MaxStep)
{
    node* x_nearest;
    node* x_new;
    node* x_min;
    point x_rand;
    point temp_x_new;
    double c_min;
    double yRRT = 1000;
    double d = 10;
    double n = 2;
    int iteration = 0;

    for (iteration = 0; iteration < K; iteration++){
        x_rand = randomState(x_max, x_min_d, y_max, y_min);
        if (iteration % 5 == 0){
            x_rand = x_goal;
        }
        x_nearest = ptrTable[nearestNeighbor(x_rand)];
        temp_x_new = newState(x_nearest->idx, x_rand, MaxStep);
        x_new = new node;
        x_new->location = temp_x_new;
        if (!isCollision(x_nearest->location, x_new->location)){
            treeNear(x_new, std::min(yRRT*pow(std::log((double)count)/(double)count,1/d),(double)n));
            addVertex(x_new->location, x_rand, -1);
            delete x_new;
            x_new = ptrTable[count-1];
            x_min=x_nearest;
            c_min=treeCost(x_nearest)+lineCost(x_nearest,x_new);
            int i = 0;
            for (i = 0;i < Xnear.size(); i++){
                if(!isCollision(Xnear[i]->location,x_new->location) && (treeCost(Xnear[i])+lineCost(Xnear[i],x_new))< c_min){
                    x_min = Xnear[i];
                    c_min=treeCost(Xnear[i])+lineCost(Xnear[i],x_new);
                }
            }
            ptrTable[x_new->idx]->idx_parent=x_min->idx;
            for (i = 0;i < Xnear.size(); i++){
                if(!isCollision(Xnear[i]->location,x_new->location) && (treeCost(x_new)+lineCost(Xnear[i],x_new))< treeCost(Xnear[i])){
                    Xnear[i]->idx_parent = x_new->idx;
                }

            }
        }
        else{
            delete x_new;
        }
    }
}

