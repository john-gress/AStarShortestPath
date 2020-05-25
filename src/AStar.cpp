//
// Compile with: g++ -std=c++11 -o AStar src/AStar.cpp
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
#define COMBINED_HEURISTIC 3
#define VISITED 2
#define UNKNOWN_NODE 99
#define UNKNOWN_DISTANCE std::numeric_limits<int>::max()

class MyCompare
{
   public:
      bool operator() (const std::tuple<int, int, int, int>& a, const std::tuple<int, int, int, int>& b)
      {
         return std::get<COMBINED_HEURISTIC>(a) > std::get<COMBINED_HEURISTIC>(b);
      }
};

typedef std::priority_queue<std::tuple<int, int, int, int>, \
   std::vector< std::tuple<int, int, int, int> >, MyCompare > DistanceNode;

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
      std::cout << "Priority " << std::get<NODE>(nodeDistance)
         << ", " << std::get<DISTANCE>(nodeDistance)
         << ", " << std::get<PATHVIA>(nodeDistance)
         << ", " << std::get<COMBINED_HEURISTIC>(nodeDistance)
         << std::endl;
   }
}

void UpdatePriorityQueue(int node, int distance, int nodeVia, int combinedHeuristic, DistanceNode& distPq) {
   DistanceNode tmpPq;
   bool found(false);
   while (!distPq.empty()) {
      const std::tuple<int, int, int, int>& distNode = distPq.top();
      if (std::get<NODE>(distNode) == node) {
         // replace node in priority queue
         tmpPq.push(std::make_tuple(node, distance, nodeVia, combinedHeuristic));
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
      tmpPq.push(std::make_tuple(node, distance, nodeVia, combinedHeuristic));
      std::cout << "   Add distNode: " << GetNodeLetter(node) << ", " << distance << ", via: "
         << GetNodeLetter(nodeVia) << ", combinedHeuristic: " << combinedHeuristic << std::endl;
   }
   distPq = tmpPq;
}

void VisitNode(int nodeId, std::vector<std::pair<int, int> >& nodeEdges,
   std::vector<std::vector<int> >& nodeHeuristic,
   int endNode, std::vector<std::tuple<int, int, bool, int> >& pathCost, DistanceNode& distPq) { 

   for (const auto edge : nodeEdges) {
      // Looking at the edges associated with nodeId; i.e., what is nodeId connected to in the graph
      if (! std::get<VISITED>(pathCost[edge.first])) {
         // Using edge as X -> Y, where X is nodeId and Y is edge.first. The combined heuristic cost is the 
         // combined cost of the shortest path calculated to Y, plus the physical distance of Y to endNode.
         // If combined heuristic cost of getting to Y, as previously calculated from startNode, is greater than
         // the combined heuristic cost of the distance from X to Y plus the cost to get from startNode to X plus
         // the physical distance of X to endNode; then a better path has been found to endNode through Y via X.
         if (std::get<COMBINED_HEURISTIC>(pathCost[edge.first])
               > edge.second + std::get<DISTANCE>(pathCost[nodeId]) + nodeHeuristic[edge.first][endNode]) {
            std::get<DISTANCE>(pathCost[edge.first]) = edge.second + std::get<DISTANCE>(pathCost[nodeId]);
            std::get<COMBINED_HEURISTIC>(pathCost[edge.first]) = 
               std::get<DISTANCE>(pathCost[edge.first]) + nodeHeuristic[edge.first][endNode];
            UpdatePriorityQueue(edge.first, std::get<DISTANCE>(pathCost[edge.first]), nodeId,
               std::get<COMBINED_HEURISTIC>(pathCost[edge.first]), distPq);
         }
      }
   }
   std::get<VISITED>(pathCost[nodeId]) = true;
}

bool AStar(std::vector<std::vector< std::pair<int, int> > >& edges, 
   std::vector<std::vector<int> >& nodeHeuristic, int startNode, int endNode,
   std::vector<std::tuple<int, int, bool, int> >& pathCost, DistanceNode& distPq,
   std::stack<std::tuple<int, int, int, int> >& path) { 

   std::cout << "Finding shortest path from: " << GetNodeLetter(startNode) << " to: " 
      << GetNodeLetter(endNode) << std::endl;

   bool searchComplete(false);
   bool pathFound = false;
   while (! searchComplete) {
      if (distPq.empty()) {
         std::cout << "Priority queue is empty before end node is found. Aborting path search." << std::endl;
         searchComplete = true;
      } else {
         std::tuple<int, int, int, int> nextNode = distPq.top();
         distPq.pop();
         int nodeId = std::get<NODE>(nextNode);

         std::cout << std::endl << "Visiting node: " <<  GetNodeLetter(nodeId) << std::endl;
         if (nodeId == endNode) {
            std::cout << "   Arrived at destination: " <<  GetNodeLetter(nodeId) << std::endl;
            searchComplete = true;
            pathFound = true;
         } else {
            VisitNode(nodeId, edges[nodeId], nodeHeuristic, endNode, pathCost, distPq);
         }
         path.push(nextNode);
      }
   }
   return pathFound;
}

void InitGraph(std::vector<std::vector< std::pair<int, int> > >& edges) {

   // The pairs in the edges:
   //    * first is the other end of the edge defined by X -> Y;
   //    * second is the cost to get from X -> Y

   // S: edges: S -> A; S -> B; S -> C
   edges.push_back({ std::make_pair(1, 7), std::make_pair(2, 2), std::make_pair(3, 3) });

   // A: edges: A -> S; A -> B; A -> D
   edges.push_back({ std::make_pair(0, 7), std::make_pair(2, 3), std::make_pair(4, 4) });

   // B: edges: B -> S; B -> A; B -> D; B -> H
   edges.push_back({ std::make_pair(0, 2), std::make_pair(1, 3), std::make_pair(4, 4), std::make_pair(8, 1) });

   // C: edges: C -> S; C -> L
   edges.push_back({ std::make_pair(0, 3), std::make_pair(12, 2) });

   // D: edges: D -> A; D -> B; D -> F
   edges.push_back({ std::make_pair(1, 4), std::make_pair(2, 4), std::make_pair(6, 5) });

   // E: edges: E -> G; E -> K
   edges.push_back({ std::make_pair(7, 2), std::make_pair(11, 5) });

   // F: edges: F -> D; F -> H
   edges.push_back({ std::make_pair(4, 5), std::make_pair(8, 3) });

   // G: edges: G -> E; G -> H
   edges.push_back({ std::make_pair(5, 2), std::make_pair(8, 2) });

   // H: edges: H -> B; H -> F; H -> G
   edges.push_back({ std::make_pair(2, 1), std::make_pair(6, 3), std::make_pair(7, 2) });

   // I: edges: I -> J; I -> K; I -> L
   edges.push_back({ std::make_pair(10, 6), std::make_pair(11, 4), std::make_pair(12, 4) });

   // J: edges: J -> I; J -> K; J -> L
   edges.push_back({ std::make_pair(9, 6), std::make_pair(11, 4), std::make_pair(12, 4) });

   // K: edges: K -> E; K -> I; K -> J
   edges.push_back({ std::make_pair(5, 5), std::make_pair(9, 4), std::make_pair(10, 4) });

   // L: edges: L -> C; L -> I; L -> J
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

void InitNodeHeuristics(std::vector<std::vector<int> >& nodeHeuristic) {
   // The heuristic is the straight-line distance from the node to every other node.
   
   // Node S
   //           distance to: S  A  B  C  D   E  F  G  H  I  J  K  L
   nodeHeuristic.push_back({ 0, 7, 2, 3, 4, 10, 7, 7, 5, 7, 8, 9, 6 });

   // Node A
   nodeHeuristic.push_back({ 3, 0, 2, 6, 2, 9, 5, 6, 4, 8, 10, 10, 9 });

   // Node B
   nodeHeuristic.push_back({ 2, 2, 0, 4, 2, 7, 4, 4, 1, 5, 7, 7, 6 });

   // Node C
   nodeHeuristic.push_back({ 3, 6, 4, 0, 5, 8, 8, 6, 5, 3, 4, 5, 2 });

   // Node D
   nodeHeuristic.push_back({ 4, 2, 2, 5, 0, 8, 2, 4, 2, 7, 9, 9, 9 });

   // Node E
   nodeHeuristic.push_back({ 10, 9, 7, 8, 8, 0, 6, 3, 6, 4, 4, 3, 6 });

   // Node F
   nodeHeuristic.push_back({ 7, 5, 4, 8, 2, 6, 0, 2, 2, 5, 6, 5, 6 });

   // Node G
   nodeHeuristic.push_back({ 7, 6, 4, 6, 4, 3, 2, 0, 2, 3, 4, 3, 4 });

   // Node H
   nodeHeuristic.push_back({ 5, 4, 1, 5, 2, 6, 2, 2, 0, 4, 7, 6, 7 });

   // Node I
   nodeHeuristic.push_back({ 7, 8, 5, 3, 7, 4, 5, 3, 4, 0, 2, 2, 2 });

   // Node J
   nodeHeuristic.push_back({ 8, 10, 7, 4, 9, 4, 6, 4, 7, 2, 0, 2, 2 });

   // Node K
   nodeHeuristic.push_back({ 9, 10, 7, 5, 9, 3, 5, 3, 6, 2, 2, 0, 3 });

   // Node L
   nodeHeuristic.push_back({ 6, 9, 6, 2, 9, 6, 6, 4, 7, 2, 2, 3, 0 });
}

void InitPathCost(std::vector<std::vector< std::pair<int, int> > >& edges,
   int startNode, int endNode, std::vector<std::vector<int> >& nodeHeuristic,
   std::vector<std::tuple<int, int, bool, int> >& pathCost) {

   // tuple is previous node, distance, visited, combined heuristic cost
   pathCost[startNode] = std::make_tuple(startNode, 0, false, nodeHeuristic[startNode][endNode]);

   for (int i = 0; i < edges.size(); i++) {
      if (i != startNode) {
         pathCost[i] = std::make_tuple(UNKNOWN_NODE, UNKNOWN_DISTANCE, false, UNKNOWN_DISTANCE);
         //std::cout << "pathCost " << GetNodeLetter(i) << ", " 
         //   <<  GetNodeLetter(std::get<NODE>(pathCost[i])) << ", " << std::get<DISTANCE>(pathCost[i]) << std::endl;
      } 
   }
}

void PrintPath(int startNode, int endNode, std::stack<std::tuple<int, int, int, int> >& path) {
   int nextNode(endNode);
   std::cout << std::endl << std::endl << "Path in reverse order:" << std::endl;
   while (! path.empty()) {
      std::tuple<int, int, int, int>& distNode = path.top();
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

   std::stack<std::tuple<int, int, int, int> > path;
   std::vector<std::vector< std::pair<int, int> > > edges;
   std::vector<std::vector<int> > nodeHeuristic;

   InitGraph(edges);
   PrintGraph(edges);
   InitNodeHeuristics(nodeHeuristic);

   std::vector<std::tuple<int, int, bool, int> > pathCost(edges.size());
   DistanceNode distPq; // distance priority queue
   distPq.push(std::make_tuple(startNode, 0, startNode, nodeHeuristic[startNode][endNode])); // start with the startNode
   InitPathCost(edges, startNode, endNode, nodeHeuristic, pathCost);

   std::cout << std::endl << std::endl;
   bool pathFound = AStar(edges, nodeHeuristic, startNode, endNode, pathCost, distPq, path);
   if (pathFound) {
      PrintPath(startNode, endNode, path);
   }
}
