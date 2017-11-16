#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
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

struct vertex {
    typedef std::pair<int, vertex*> ve;
    vector<ve> adj; //cost of edge, destination vertex
    string name;
    vertex(string s) : name(s) {}
};

class Graph{
public:
    typedef std::map<std::string, vertex *> vmap;
    vmap work;
    void addvertex(const std::string&);
    void addedge(const std::string& from, const std::string& to, double cost);
}
void graph::addvertex(const std::string &name)
{
    vmap::iterator itr = work.find(name);
    if (itr == work.end())
    {
        vertex *v;
        v = new vertex(name);
        work[name] = v;
        return;
    }
    std::cout << "\nVertex already exists!";
}

void graph::addedge(const std::string& from, const std::string& to, double cost)
{
    vertex *f = (work.find(from)->second);
    vertex *t = (work.find(to)->second);
    std::pair<int, vertex *> edge = make_pair(cost, t);
    f->adj.push_back(edge);
}