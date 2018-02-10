#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>

#include <signal.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <string>
#include <unistd.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <deque>
#include <queue>
#include <math.h>
#include <vector>

using namespace std;

struct vertex {
    //typedef pair<int, vertex*> ve;
    //vector<ve> adj; //cost of edge, destination vertex
    string name;
    string password;
    int id;
    int x;
    int y;
    vertex(string s, string pass, int posx, int posy, int count) {
        name = s;
        password = pass;
        x = posx;
        y = posy;
        id = count;
    }
};

class Graph{
public:
    int count;
    Graph(){
        count=0;
    }
    typedef map<string, vertex*> vmap;  //vertex map
    vmap work;

    vector<vector<double> > floyd;
    vector<vector<int> > path;

    void addvertex(const string&, const string&, const int&, const int&);
    void addedge(const int&, const int&, double);
    void pathfindingFloyd();
    queue<vertex> returnPath(const string&, const string&);
    void findpath(vector<int>&, vector<vector<int> >&, int, int);
    string closestNode(double, double);
};

void Graph::addvertex(const string &ssid, const string &password, const int &posx, const int &posy)
{
    vertex *v;
    v = new vertex(ssid, password, posx, posy, count);
    work[ssid] = v;
    count++;
    return;
}

void Graph::addedge(const int& from, const int& to, double cost)
{
    floyd[from-1][to-1] = cost;
    floyd[to-1][from-1] = cost;
}

void Graph::pathfindingFloyd(){
    int size = count;
    
    for(int i=0; i<size; i++){
        for(int j=0; j<size; j++){
            for(int k=0; k<size; k++){
                if(floyd[j][i] + floyd[i][k] < floyd[j][k]){
                    path[j][k] = i;
                    floyd[j][k] = floyd[j][i] + floyd[i][k];
                }
            }
        }
    }
}

queue<vertex> Graph::returnPath(const string& from, const string& to){
    vertex *f= (work.find(from)->second);
    vertex *t= (work.find(to)->second);
    queue<vertex> result;
    vector<int> temp;
    map<string, vertex*>::iterator it = work.begin();
    cout<< "Generated path consists of : ";
    result.push(*f);
    cout<< f->name << " -> ";

    findpath(temp, path, f->id, t->id);

    for(int i = 0; i<temp.size(); i++){
        for(it = work.begin(); it!=work.end(); ++it){
            if(temp[i] == it->second->id ){
                result.push(*(it->second));
                cout<< it->first <<endl;
            }
        }
    }
    
    result.push(*t);
    cout << t->name << "end" <<endl;
/*
    for(int i=0; i<result.size(); i++){
        cout<<result[i]<<endl;
    }
    */
    return result;
}

void Graph::findpath(vector<int> &result, vector<vector<int> > &P, int q, int r){
    if(P[q][r] != 0){
        findpath(result, P, q, P[q][r]);
        result.push_back(P[q][r]);
        findpath(result, P, P[q][r], r);
    }
}

string Graph::closestNode(double cur_x, double cur_y){
    map<string, vertex*>::iterator it;
    vertex *f;
    double min_dist = 1000000;
    double x, y;

    f = work.begin()->second;
    for(it=work.begin(); it!=work.end(); ++it){
        x = it->second->x;
        y = it->second->y;
        //cout<< it->second->x << it->second->y << endl;
        if(min_dist > sqrt(pow(cur_x-x,2) + pow(cur_y-y,2))){
            min_dist = sqrt(pow(cur_x-x,2) + pow(cur_y-y,2));
            f = it->second;
        }
    }

    return f->name;
}