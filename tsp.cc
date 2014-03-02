// -*- C++ -*-

/* Copyright (c) 2014 Powell Molleti
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/**
 * @file:tsp.cc
 * Implementation Shortest path between given points starting from a point.
 * Default is dynamic programming version uses c++11 unordered map for space.
 * Options:
 * --slow:
 *   Runs the naive n! implementation
 *
 */

#include <iostream>
#include <vector>
#include <list>
#include <queue>
#include <stack>
#include <set>
#include <unordered_map>
#include <string>
#include <sstream>
#include <fstream>
#include <cctype>
#include <cstdlib>
#include <cerrno>
#include <cstring>
#include <cmath>
#include <sys/stat.h>

#include <algorithm>

using namespace std;

#define NDEBUG 0  // 1 for more debug.

const unsigned int maxFSize = 1024*1024*1024;     /// Max File in Bytes, 1GB
const unsigned int maxLines = 10000;              /// Max number of Lines

/**
 * Class for each Node with x,y co-ordinate and store both
 * full node details and a short form so desired output can be
 * printed.
 */
class Node {
public:
    Node();
    Node(string& n, double x, double y);
    ~Node();
    double getDist(Node &n2);
    string name_;
    string short_;
    double x_;
    double y_;
};

Node::Node()
    : name_(""), short_(""), x_(0.0), y_(0.0)
{}
Node::Node(string& s, double x, double y)
    : name_(s), short_(""), x_(x), y_(y)
{
#if NDEBUG
    cout.precision(10);

    cout << "Point : " << name_ << " X :" << x_
          << " Y :" << y_ << endl;
#endif

    // cut the name to short form
    for (size_t i = 0; i < name_.length(); i++) {
        if (name_.data()[i] == '|') {
            break;
        }
        short_.append(&name_.data()[i], 1);
    }

    int v = atoi(short_.c_str());
    stringstream ss;
    ss << v;
    short_ = ss.str();
}

Node::~Node()
{}

double Node::getDist(Node &n2)
{
    double xd = abs(x_ - n2.x_);
    double yd = abs(y_ - n2.y_);
    return(sqrt((xd*xd + yd*yd)));
}

/**
 * TNode helps with implementing n! version of TSP.
 * Contains all the edges starting from 1.
 */
class TNode {
public:
    TNode();
    TNode(Node *n, double d);
    ~TNode();

    TNode* addEdge(Node *n);          /// Add a new edge and return its pointer
    double addToTotal(double d);      /// Add to the totald_
    void setDist(double d);           /// Set d
    void setParent(TNode *p);         /// We set parent link help us chart
                                      /// back the shortest path.
    Node *n_;             // Me
    double d_;            // Distance from parent.
    double totald_;       // Total distance from root to this edge.
    TNode *p_;            // Pointer to parent
    list<TNode *> *elist_; // Edges.
};

TNode::TNode()
    : n_(NULL), d_(0.0), totald_(0.0), p_(NULL), elist_(NULL)
{}

TNode::TNode(Node *n, double d)
    : n_(n), d_(d), totald_(0.0), p_(NULL), elist_(NULL)
{}

TNode::~TNode()
{
    elist_->clear();
    delete elist_;
    elist_ = NULL;
}

/**
 * Add the node as an edge , will return the pointer
 * to the new TNode added as edge.
 */
TNode* TNode::addEdge(Node *n)
{
    if (!elist_) {
        elist_ = new list<TNode *>;
    }

    double d = n_->getDist(*n);
#if NDEBUG
    cout << n_->name_
         << " -- " << n->name_ << " : "
         << d << endl;
#endif
    TNode *t = new TNode(n, d);
    t->addToTotal(this->totald_);
    t->setParent(this);
    elist_->push_back(t);
    return t;
}


double TNode::addToTotal(double d)
{
    double prev = totald_;
    totald_ = d_ + d;
    return(prev);
}

void TNode::setDist(double d)
{
    d_ = d;
}

/**
 * This link helps us traverse back to root give
 * the last edge of the shortest path.
 */
void TNode::setParent(TNode *p)
{
    p_ = p;
}

/**
 * Helps with sorting all paths, helps with debugging.
 */
inline bool cmp (const TNode *n1, const TNode *n2)
{
    return n1->totald_ < n2->totald_;
}

/**
 * This class helps us maintain the hash map for look up of
 * minimum path given a set of nodes.
 * @name_
 *    eg. ({1}{2}{3}{4}) - with root as 1 get for all paths of {2}{3}{4}
 * @minPath_
 *    eg. {3}{4}{2} - The shortest path result.
 * @minDist_
 *    eg. 392.39292 - The distance of the shortest path.
 */
class MinNode {
public:
    MinNode();
    MinNode(string n, string mpath, double d);
    ~MinNode();

    string name_;      // all combinations of elements in name
    string minPath_;   // Final minimum path.
    double minDist_;   // mininum distance for all paths.
};

MinNode::MinNode()
    : name_(""), minPath_(""), minDist_(0.0)
{}

MinNode::~MinNode()
{}

MinNode::MinNode(string n, string mpath, double d)
    : name_(n), minPath_(mpath), minDist_(d)
{}


/**
 * Abstraction to help us get best route from given input.
 * Works in two modes.
 * O(n!) -
 *    This implementation builds a N-way tree and gets optimal path.
 * O(2^n*n^2) -
 *    This implementation uses hash map to to help with not computing path
 *    distance for the same path again and again.
 * Input format -
 *   1 | CodeEval 1355 Market St, SF (37.7768016, -122.4169151)
 *   2 | Yelp 706 Mission St, SF (37.7860105, -122.4025377)
 *   3 | Square 110 5th St, SF (37.7821494, -122.4058960)
 *   4 | Airbnb 99 Rhode Island St, SF (37.7689269, -122.4029053)
 *   5 | Dropbox 185 Berry St, SF (37.7768800, -122.3911496)
 *   6 | Zynga 699 8th St, SF (37.7706628, -122.4040139)
 */
class HappyDriver {
public:
    HappyDriver();
    ~HappyDriver();
    HappyDriver(string fname, bool faster = true);

    int parseInputFile();
    int setDriveMode(bool faster=true);
    void setDebug(bool debug);
    void setMaxLines(unsigned int maxLines);

    // Main call to get shortest tour.
    double reduceCommute(stringstream& route);

private:
    // parsing helpers.
    int parse_line(string &line);
    int parse_input(ifstream &infile);

    // Tree helpers.
    int insertNode(TNode *t, queue<Node *> q, TNode *root);
    double runFactorial(stringstream& route);

    // Dynamic mode helpers.
    string getHashInput(Node *n, queue<Node *>q);
    double getMinFromHash(string path, string& out);
    bool isOutValid(string s);
    double getMinFromInput(Node *n, queue<Node *>q, string &out);
    double setMinForInput(Node *n, queue<Node *>q, string &out);

    // Dynamic Programming algo
    double getMin(Node *n, queue<Node *>q, string &retq);
    double getTour(stringstream& tour);

    string fname_;               /// Input filename
    unsigned int maxLines_;      /// Max lines
    bool faster_;                /// Algo mode, true for dynamic, false for n!
    bool debug_;                 /// set for debug mode.

    vector<Node> nodeList;       /// Input nodes
    unordered_map<string, MinNode> minHashMap; /// lookup for dynamic mode
    TNode *tspRoot;                            /// Tree for n!, no full tree
                                               /// in dynamic mode just
                                               /// the root.
    TNode *sPathEdge;                          /// n! shortest Path edge.
    vector<TNode *> edgeList;                  /// For debugging.
};

HappyDriver::HappyDriver()
    : fname_(""), maxLines_(maxLines), faster_(true),
      debug_(false), tspRoot(NULL), sPathEdge(NULL)
{}

HappyDriver::~HappyDriver()
{}

HappyDriver::HappyDriver(string fname, bool faster)
    : fname_(fname), maxLines_(maxLines), faster_(faster),
      debug_(false), tspRoot(NULL), sPathEdge(NULL)
{
}

void HappyDriver::setDebug(bool debug)
{
    debug_ = debug;
}

void HappyDriver::setMaxLines(unsigned int maxLines)
{
    maxLines_ = maxLines;
    return;
}

int HappyDriver::insertNode(TNode *t, queue<Node *> q, TNode *root)
{
    if (q.size() == 0) { // here is the last edge
        TNode *last_edge = t;
#if NDEBUG
        cout << "Weight : " << last_edge->totald_ << endl;
#endif
        if (!sPathEdge || sPathEdge->totald_ > last_edge->totald_) {
            sPathEdge = last_edge;
        }

        if (debug_) {
            edgeList.push_back(last_edge);
        }
        return(0);
    }

    int qsize = q.size();
    int count = 0;
    while(count < qsize)
    {
        Node *n = q.front();
        q.pop();
        TNode *tn = t->addEdge(n);
        count++;
        insertNode(tn, q, root);
        q.push(n);
    }

    return (0);
}

string HappyDriver::getHashInput(Node *n, queue<Node *>q)
{
    string ret = "{";
    ret.append(n->short_);
    ret.append("}");

    while(!q.empty()){
        Node *t = q.front();
        q.pop();
        ret.append("{");
        ret.append(t->short_);
        ret.append("}");
    }

    return(ret);
}

double HappyDriver::getMinFromHash(string path, string& out)
{
    unordered_map<string, MinNode>::iterator it
        = minHashMap.find(path);

    double d = 0.0;
    if (it != minHashMap.end()) {
        MinNode &n = it->second;
        out = n.minPath_;
        d = n.minDist_;
#if NDEBUG
        cout << "Hit : " << path << " d: "
             << d << " Path: " << out << endl;
#endif
        return(d);
    }

    // could not find.
    out = "NULL";
    return(d);
}

bool HappyDriver::isOutValid(string s)
{
    if (s == "NULL" || s.length() == 0) {
        return false;
    }

    return true;
}

double HappyDriver::getMinFromInput(Node *n, queue<Node *>q, string &out)
{
    string i = getHashInput(n, q);
    double d = getMinFromHash(i, out);
    return(d);
}

/**
 * call only when not found in hashMap,
 * does not check if something exists!.
 */
double HappyDriver::setMinForInput(Node *n, queue<Node *>q, string &out)
{
    string hin = "{";
    hin.append(n->short_);
    hin.append("}");

    if (q.empty()) {
        return(0.0);
    }

    Node *t = q.front();
    q.pop();
    double d = n->getDist(*t);

    out.append("{");
    out.append(t->short_);
    out.append("}");

    while(!q.empty()) {
        Node *tt = q.front();
        q.pop();
        d += t->getDist(*tt);

        out.append("{");
        out.append(tt->short_);
        out.append("}");
        t = tt;
    }

    hin.append(out);
    MinNode mn(hin, out, d);
    pair<string, MinNode> p(hin, mn);
    pair<unordered_map<string, MinNode>::iterator,bool> mret =
        minHashMap.insert(p);
    if (mret.second == false) {
        cerr << "Error minHashMap insert of " << hin << endl;
        return(0.0);
    }

#if NDEBUG
    cout << "inserted : " << hin << " distance : " << d
         << " min Path :" << d << endl;
#endif
    return(d);
}

/**
 * Dynamic Programming, uses Hash Map for space.
 * Uses recursion for now.
 * TODO:
 * Check performance gains with iterative solution.
 */
double HappyDriver::getMin(Node *n, queue<Node *>q, string &retq)
{
    int qsize = q.size();

    if (qsize == 1) {
        // set of two elements?, get it or store it!.
        double d = getMinFromInput(n, q, retq);
        if (isOutValid(retq)) {
            return(d);
        }
        retq = "";
        d = setMinForInput(n, q, retq);

        // insert other direction too!.
        {
            string useless;
            Node *t = q.front();
            queue<Node *> qt;
            qt.push(n);
            setMinForInput(t, qt, useless);
        }

        return(d);
    }

    int count = 0;
    double mindist = 0.0;
    string minpath = "";
    while(count < qsize)
    {
        string rpath = "";
        Node *t = q.front();
        q.pop();
        count++;
        double d = n->getDist(*t);
        double dmin = getMinFromInput(t, q, rpath);
        if (isOutValid(rpath) == false) {
            // could not find.
            rpath = "";
            d = d + getMin(t, q, rpath);
        } else {
            d = d + dmin;
        }

        if (!mindist || mindist > d) {
            mindist = d;
            minpath = "";
            minpath.append("{");
            minpath.append(t->short_);
            minpath.append("}");
            minpath.append(rpath);
        }
        q.push(t);
    }

    string hin = getHashInput(n, q);
    retq = minpath;
    MinNode mn(hin, retq, mindist);
    pair<string, MinNode> p(hin, mn);
    pair<unordered_map<string, MinNode>::iterator,bool> mret =
        minHashMap.insert(p);
    if (mret.second == false) {
        cerr << "Error minHashMap insert of " << hin
             << endl;
        return(0.0);
    }

#if NDEBUG
    cout << "inserted : " << hin << " distance : " << mindist
         << " min Path : " << retq << endl;
#endif

    return(mindist);
}

/**
 * Dyanamic Programming entry point.
 * Returns the shortest path distance and also
 * output as desired.
 */
double HappyDriver::getTour(stringstream& tour)
{
    // return nothing on a empty set.
    if (nodeList.size() == 0) {
        return(0.0);
    }

    Node *r = &nodeList[0];
    TNode *tspRoot = new TNode(r, 0.0);

    // Now that we have the root use index 1 to N and build
    queue<Node*> *q = new queue<Node*>; // on heap
                                        // so we support large values

    // we have items in the queue other than root.
    // combinatrics :)
    for (size_t i = 1; i < nodeList.size(); i++) {
        q->push(&nodeList[i]);
    }

    string out = "";
    double d = getMin(tspRoot->n_, *q, out);
    string path("{");
    path.append(tspRoot->n_->short_);
    path.append("}");
    path.append(out);

    // convert the string to output.
    string::iterator it = path.begin();
    bool is_node = false;
    string tmp = "";
    for (;it != path.end(); ++it) {
        char t = *it;
        if (t == '}') {
            is_node = false;
            tour << tmp << endl;
            tmp = "";
            continue;
        }

        if (t == '{') {
            is_node = true;
            continue;
        }

        if (is_node) {
            t = *it;
            tmp.append(&t, 1);
            continue;
        }
    }

    return(d);
}

/**
 * Naive implementations O(n!).
 * Uses recursion to build a full combinatrics tree.
 */
double HappyDriver::runFactorial(stringstream& route)
{
    // return nothing on a empty set.
    if (nodeList.size() == 0) {
        return(0.0);
    }

    // we have to start from first point, that will be the root.
    Node *r = &nodeList[0];
    TNode *tspRoot = new TNode(r, 0.0);

    // If there is only one point then return and bail.
    if (nodeList.size() == 1) {
        route << r->short_ << endl;
        return(0.0);
    }

    // Now that we have the root use index 1 to N and build
    // the TSP tree and keep calculating total d_
    queue<Node*> *q = new queue<Node*>; // on heap
                                        // so we support large values


    // we have items in the queue other than root.
    // combinatrics :)
    for (size_t i = 1; i < nodeList.size(); i++) {
        q->push(&nodeList[i]);
    }

    int qsize = q->size();
    int count = 0;
    while(count < qsize) {
        Node *n = q->front();
        q->pop();
        count++;
        TNode *tn = tspRoot->addEdge(n);
        insertNode(tn, *q, tspRoot);
        q->push(n);
    }

    // if debug print all paths in increased total distance.
    if (debug_) {
        sort(edgeList.begin(), edgeList.end(), cmp);
        // print the weights.
        count = 0;
        vector<TNode *>::iterator it = edgeList.begin();
        for (; it != edgeList.end(); ++it)
        {
            TNode *t = *it;
            cout << " Total Distance [" << ++count << "]: "
                 << t->totald_ << " :PATH: ";
            stack<TNode *> st;
            while(t != tspRoot) {
                st.push(t);
                t=t->p_;
            }
            st.push(t);
            while(!st.empty())
            {
                TNode *r = st.top();
                st.pop();
                cout << r->n_->short_ << " ";
            }
            cout << endl;
            break;
        }
        cout << endl;
    }

    TNode *t = sPathEdge;
    stack<TNode *> st;
    while(t != tspRoot) {
        st.push(t);
        t=t->p_;
    }
    st.push(t);

    while(!st.empty())
    {
        TNode *r = st.top();
        st.pop();
        route << r->n_->short_ << endl;
    }

    return(sPathEdge->totald_);
}

double HappyDriver::reduceCommute(stringstream& route)
{
    if (faster_ == false) {
        return(runFactorial(route));
    }

    return(getTour(route));
}

/**
 * get the point name with and x and y cordinates
 */
int HappyDriver::parse_line(string &line)
{
    string::iterator it = line.begin();

    int state = 0;
    string point = "";
    string x = "";
    string y = "";
    for (; it != line.end(); ++it) {
        char c = *it;

        if (c == '(') {
            state = 1; // get X cordinate
            continue;
        }

        if (state >= 1 && c == ')') {
            state = 3; // start and end of bracket
            break;
        }

        if (state >= 1 && state < 3 && c == ',') {
            state = 2; // get Y cordinate.
        }

        if (!state) {
            point.append(&c, 1);
            continue;
        }

        if (state == 1) {
            if (isdigit(c) || c == '.' || c == '+' || c == '-')
                x.append(&c, 1);
            continue;
        }

        if (state == 2) {
            if (isdigit(c) || c == '.' || c == '+' || c == '-')
                y.append(&c, 1);
            continue;
        }

        if (state == 3)
            break;
    }

    if (state != 3) {
        cout << "Could not get parse , point: " << point
             << "x : " << x << " y : " << y << endl;
        return(-1);
    }

    if (x.length() == 0 || y.length() == 0) {
        cout << " Could not get x or y"
             << "point: " << point << "x : " << x << " y : " << y << endl;
        return(-1);
    }
    double xd = 0.0;
    double yd = 0.0;

    xd = strtod(x.c_str(), NULL);
    if (errno == ERANGE) {
        cout << "X is bad :" << x << " , " << strerror(errno) << endl;
        return(-1);
    }

    if (xd == 0) { // did we convert?
    }

    yd = strtod(y.c_str(), NULL);
    if (errno == ERANGE) {
        cout << "Y is bad :" << y << " , " << strerror(errno) << endl;
        return(-1);
    }

    if (yd == 0) { // did we convert?
    }

    Node n(point, xd, yd);
    nodeList.push_back(n);
    return(0);
}

/**
 * Parse the file for all points.
 */
int HappyDriver::parse_input(ifstream &infile)
{
    string line ="";
    unsigned int line_count = 0;
    while(!infile.eof()) {
        getline(infile, line);
        if (++line_count > maxLines) {
            cout << "Cannot parse more than " << maxLines
                 << " lines" << endl;
            return(-1);
        }

        if (line.length() <= 1)
            continue;

        // we have a parsable line.
        // get data from it.
        //cout << "[" << line_count << "] : " << line << endl;

        if (parse_line(line) < 0) {
            cout << "Could not parse line [ " << line_count
                 << "]: " << line << endl;
            return(-1);
        }
    }
    return(0);
}

/**
 * Returns error on parse error.
 */
int HappyDriver::parseInputFile()
{
    ifstream infile;

    if (fname_.length() == 0)
        return(-1);

    infile.open(fname_, ifstream::in);
    if (!infile.good()) {
        cout << "Invalid file : " << fname_ << endl;
        return(-1);
    }

    // file is valid.
    int ret = parse_input(infile);
    if (ret < 0) {
        cout << "Error parsing" << endl;
        infile.close();
        return(-1);
    }
    infile.close();
    return(0);
}

/**
 * Helps with stopping bad filenames and files that exceed the limit
 * we expect.
 */
int fileCheck(const string& fname)
{
    struct stat sbuf;
    memset(&sbuf, 0, sizeof(sbuf));

    if (!fname.length()) {
        cerr << "Invalid filename" << endl;
        return(-1);
    }

    if (::stat(fname.c_str(), &sbuf) == -1) {
        cerr << "stat Error for fname : " << fname << " "
             << strerror(errno) << endl;
        return(-1);
    }

    if (!S_ISREG(sbuf.st_mode)) {
        cerr << "fname : " << fname
             << " is not a regular file" << endl;
        return(-1);
    }

    if (sbuf.st_size > maxFSize) {
        cerr << "fname : " << fname
             << " size : " << sbuf.st_size
             << " exceeded max size : " << maxFSize << endl;
        return(-1);
    }
    return 0;
}

int main(int argc, char *argv[])
{
    if (argc < 2) {
        cout << "Usage: " << argv[0]
             << " <file with x,y cordinates" << endl;
        return(-1);
    }

    bool fast = true;
    if (argc == 3) {
        string opt = argv[2];
        if (opt == "--slow") {
            fast = false;
        }
    }

    // Let us do some checks to make sure we have valid input file.
    if (fileCheck(argv[1]) < 0) {
        return(-1);
    }

    HappyDriver happy_driver(argv[1], fast);

    // its a valid file, lets try to see if we have valid data.
    int ret = happy_driver.parseInputFile();
    if (ret < 0) {
        cerr << "Could not parse input file: " << argv[1] << endl;
        return(-1);
    }

    // Now we have list of points, get the best commute
    stringstream tour;
    ret = happy_driver.reduceCommute(tour);
    if (ret < 0.0) {
        cout << "Error happy_driving!!" << endl;
        return(-1);
    }

    cout << tour.str();

    return(0);
}
