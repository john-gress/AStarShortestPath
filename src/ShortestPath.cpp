//
// Compile with: g++ -std=c++11 -o ShortestPath src/ShortestPath.cpp
//
#include <iostream>
#include <vector>
#include <queue>
#include <tuple>
#include <stack>
#include <limits>
#define SIZE 13 
#define NODE 0
#define DISTANCE 1
#define PATHVIA 2
#define VISITED 2
#define UNKNOWN_NODE 99
#define UNKNOWN_DISTANCE std::numeric_limits<int>::max()

class MyCompare
{
   public:
      bool operator() (const std::tuple<int, int, int>& a, const std::tuple<int, int, int>& b)
      {
         return std::get<DISTANCE>(a) > std::get<DISTANCE>(b);
      }
};

typedef std::priority_queue<std::tuple<int, int, int>, std::vector< std::tuple<int, int, int> >, MyCompare > DistanceNode;

int ConvertNodeLetter(char* letter) {
   switch (letter[0]) {
      case 'S':
         return 0;
         break;
      case 'A':
         return 1;
         break;
      case 'B':
         return 2;
         break;
      case 'C':
         return 3;
         break;
      case 'D':
         return 4;
         break;
      case 'E':
         return 5;
         break;
      case 'F':
         return 6;
         break;
      case 'G':
         return 7;
         break;
      case 'H':
         return 8;
         break;
      case 'I':
         return 9;
         break;
      case 'J':
         return 10;
         break;
      case 'K':
         return 11;
         break;
      case 'L':
         return 12;
         break;
      default:
         return -1;
         break;
   }
}

char GetNodeLetter(int i) {
   switch (i) {
      case 0:
         return 'S';
         break;
      case 1:
         return 'A';
         break;
      case 2:
         return 'B';
         break;
      case 3:
         return 'C';
         break;
      case 4:
         return 'D';
         break;
      case 5:
         return 'E';
         break;
      case 6:
         return 'F';
         break;
      case 7:
         return 'G';
         break;
      case 8:
         return 'H';
         break;
      case 9:
         return 'I';
         break;
      case 10:
         return 'J';
         break;
      case 11:
         return 'K';
         break;
      case 12:
         return 'L';
         break;
      default:
         return 'Z';
         break;
   }
}

// helper function for debugging priority queue
void PrintPriorities(DistanceNode& distPq) { 
   while (!distPq.empty()) {
      auto nodeDistance = distPq.top();
      distPq.pop();
      std::cout << "Priority " << std::get<NODE>(nodeDistance) << ", " 
         << std::get<DISTANCE>(nodeDistance) << ", " << std::get<PATHVIA>(nodeDistance) << std::endl;
   }
}

void UpdatePriorityQueue(int node, int distance, int nodeVia, DistanceNode& distPq) {
   DistanceNode tmpPq;
   bool found(false);
   while (!distPq.empty()) {
      const std::tuple<int, int, int>& distNode = distPq.top();
      if (std::get<NODE>(distNode) == node) {
         // replace node in priority queue
         tmpPq.push(std::make_tuple(node, distance, nodeVia));
         std::cout << "   Update distNode: " << GetNodeLetter(node) << ", " << distance << ", via: "
            << GetNodeLetter(nodeVia) << std::endl;
         found = true;
      } else {
         // keep existing node in priority queue
         tmpPq.push(distNode);
      }
      distPq.pop();
   }
   if (!found) {
      tmpPq.push(std::make_tuple(node, distance, nodeVia));
      std::cout << "   Add distNode: " << GetNodeLetter(node) << ", " << distance << ", via: "
         << GetNodeLetter(nodeVia) << std::endl;
   }
   distPq = tmpPq;
}

void VisitNode(int nodeId, std::vector<std::pair<int, int> >& nodeEdges,
   int endNode, std::vector<std::tuple<int, int, bool> >& pathCost, DistanceNode& distPq) { 

   std::cout << std::endl << "Visiting node: " <<  GetNodeLetter(nodeId) << std::endl;

   for (const auto edge : nodeEdges) {
      if (! std::get<VISITED>(pathCost[edge.first])) {
         if (std::get<DISTANCE>(pathCost[edge.first]) > edge.second + std::get<DISTANCE>(pathCost[nodeId])) {
            std::get<DISTANCE>(pathCost[edge.first]) = edge.second + std::get<DISTANCE>(pathCost[nodeId]);
            UpdatePriorityQueue(edge.first, std::get<DISTANCE>(pathCost[edge.first]), nodeId, distPq);
         }
      }
   }
   std::get<VISITED>(pathCost[nodeId]) = true;
}

bool ShortestPath(std::vector<std::vector< std::pair<int, int> > >& edges, int startNode, int endNode,
   std::vector<std::tuple<int, int, bool> >& pathCost, DistanceNode& distPq,
   std::stack<std::tuple<int, int, int> >& path) { 

   std::cout << "Finding shortest path from: " << GetNodeLetter(startNode) << " to: " 
      << GetNodeLetter(endNode) << std::endl;

   bool searchComplete(false);
   bool pathFound = false;
   while (! searchComplete) {
      if (distPq.empty()) {
         std::cout << "Priority queue is empty before end node is found. Aborting path search." << std::endl;
         searchComplete = true;
      } else {
         std::tuple<int, int, int> nextNode = distPq.top();
         distPq.pop();
         int nodeId = std::get<NODE>(nextNode);

         if (nodeId == endNode) {
            searchComplete = true;
            pathFound = true;
         } else {
            VisitNode(nodeId, edges[nodeId], endNode, pathCost, distPq);
         }
         path.push(nextNode);
      }
   }
   return pathFound;
}

void InitGraph(std::vector<std::vector< std::pair<int, int> > >& edges) {
   // edge 0 -> X  (Start -> X)
   edges.push_back({ std::make_pair(1, 7), std::make_pair(2, 2), std::make_pair(3, 3) });
   // edge 1 -> X  (A -> X)
   edges.push_back({ std::make_pair(0, 7), std::make_pair(2, 3), std::make_pair(4, 4) });
   // edge 2 -> X  (B -> X)
   edges.push_back({ std::make_pair(0, 2), std::make_pair(1, 3), std::make_pair(4, 4), std::make_pair(8, 1) });
   // edge 3 -> X  (C -> X)
   edges.push_back({ std::make_pair(0, 3), std::make_pair(12, 2) });
   // edge 4 -> X  (D -> X)
   edges.push_back({ std::make_pair(1, 4), std::make_pair(2, 4), std::make_pair(6, 5) });

   // edge 5 -> X  (E -> X)
   edges.push_back({ std::make_pair(7, 2), std::make_pair(11, 5) });

   // edge 6 -> X  (F -> X)
   edges.push_back({ std::make_pair(4, 5), std::make_pair(8, 3) });
   // edge 7 -> X  (G -> X)
   edges.push_back({ std::make_pair(5, 2), std::make_pair(8, 2) });
   // edge 8 -> X  (H -> X)
   edges.push_back({ std::make_pair(2, 1), std::make_pair(6, 3), std::make_pair(7, 2) });
   // edge 9 -> X  (I -> X)
   edges.push_back({ std::make_pair(10, 6), std::make_pair(11, 4), std::make_pair(12, 4) });
   // edge 10 -> X  (J -> X)
   edges.push_back({ std::make_pair(9, 6), std::make_pair(11, 4), std::make_pair(12, 4) });
   // edge 11 -> X  (K -> X)
   edges.push_back({ std::make_pair(5, 5), std::make_pair(9, 4), std::make_pair(10, 4) });
   // edge 12 -> X  (L -> X)
   edges.push_back({ std::make_pair(3, 2), std::make_pair(9, 4), std::make_pair(10, 4) });
}

void PrintGraph(std::vector<std::vector< std::pair<int, int> > >& edges) {
   std::cout << std::endl << "Graph definition:" << std::endl;
   for (int i = 0; i < edges.size(); i++) {
      for (auto itTo : edges[i]) {
         std::cout << GetNodeLetter(i) << " -> " << GetNodeLetter(itTo.first) << " (" << itTo.second << "), ";
      }
      std::cout << std::endl;
   }
}

void InitPathCost(std::vector<std::vector< std::pair<int, int> > >& edges,
   int startNode, std::vector<std::tuple<int, int, bool> >& pathCost) {

   // tuple is previous node, distance, and visited
   pathCost[startNode] = std::make_tuple(startNode, 0, false);

   for (int i = 0; i < edges.size(); i++) {
      if (i != startNode) {
         pathCost[i] = std::make_tuple(UNKNOWN_NODE, UNKNOWN_DISTANCE, false);
         //std::cout << "pathCost " << GetNodeLetter(i) << ", " 
         //   <<  GetNodeLetter(std::get<NODE>(pathCost[i])) << ", " << std::get<DISTANCE>(pathCost[i]) << std::endl;
      } 
   }
}

void PrintPath(int startNode, int endNode, std::stack<std::tuple<int, int, int> >& path) {
   int nextNode(endNode);
   std::cout << std::endl << std::endl << "Path in reverse order:" << std::endl;
   while (! path.empty()) {
      std::tuple<int, int, int>& distNode = path.top();
      if (std::get<NODE>(distNode) == nextNode) {
         std::cout << GetNodeLetter(std::get<NODE>(distNode)) << " <-- "
            << GetNodeLetter(std::get<PATHVIA>(distNode))
            << ", " << std::get<DISTANCE>(distNode) << std::endl;
         nextNode = std::get<PATHVIA>(distNode);
      } /* else {
         std::cout << "Unused node: " << GetNodeLetter(std::get<NODE>(distNode)) << " <-- "
            << GetNodeLetter(std::get<PATHVIA>(distNode))
            << ", " << std::get<DISTANCE>(distNode) << std::endl;
      }
      */
      path.pop();
      if (nextNode == startNode) {
         break;
      }
   }
}

int main(int argc, char* argv[]) {
   if (argc != 3) { // We expect 3 arguments: the program name, the startNode and the endNode
      std::cerr << "Usage: " << argv[0] << " <startNode> <endNode>" << std::endl;
      return 1;
   }

   int startNode = ConvertNodeLetter(argv[1]);
   int endNode = ConvertNodeLetter(argv[2]);
   if (startNode == -1 || endNode == -1) {
      std::cerr << "startNode or endNode is out of range" << std::endl;
      return 1;
   }

   std::stack<std::tuple<int, int, int> > path;
   std::vector<std::vector< std::pair<int, int> > > edges;

   InitGraph(edges);
   PrintGraph(edges);

   std::vector<std::tuple<int, int, bool> > pathCost(edges.size());
   DistanceNode distPq; // distance priority queue
   distPq.push(std::make_tuple(startNode, 0, startNode)); // start with the startNode
   InitPathCost(edges, startNode, pathCost);

   std::cout << std::endl << std::endl;
   bool pathFound = ShortestPath(edges, startNode, endNode, pathCost, distPq, path);
   if (pathFound) {
      PrintPath(startNode, endNode, path);
   }
}
