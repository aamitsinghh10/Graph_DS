# Graph-Important-Problems
Introduction:
Graph is a data structure which has two components:
Node/Vertex
Edge (which connects two nodes)
Types of graph:
Directed Graph
Undirected Graph
Degree in a graph:
The number of edges that are incoming or outgoing in a graph.
In an undirected graph there are degerees.
Total degree of all the nodes = 2*edges.
In a directed graph there are:
Indegree (Incoming edge)
Outdegree (Outgoing edge)
Path:
Path is a sequence of nodes or vertex such that none of the nodes are repeating or visited more than once.
Cyclic graph:
If there is a cycle in an undirected graph we can call that as an undirected cyclic graph.
Undirected acylic graph: Tree/no cycle.
If there is a cycle in a directed graph we can call that as a directed cyclic graph.
Directed acyclic graph: No cycle.
Weighted graph:
Weighted directed graph.
Weighted undirected graph.
Graph representation in C++:
Adjacency matrix:

Find out the number of nodes.
Check if it is one based indexing or zero based indexing.
If it is one based indexing we create (N+1)*(N+1) 2D array.
Fill the 2D array with zeroes.
Iterate over all the edges and mark 1 whenever there is an edge present for that node. (adj[u][v] =1)
If the graph is undirected, mark the reverse edge 1 as well. (adj[u][v] =1) and (adj[v][u] =1)
Disadvantages of using this method:
We can only use adjacency matrix for smaller values of n.
Space complexity: O(n*n)
Adjacency List:

If it's an undirected or directed graph, we are going to have vector of an adjaceny array of size (n+1).
For directed:

adj[u].push_back(v);
For undirected:

adj[u].push_back(v);
adj[v].push_back(u);
Then we can store adjacent nodes at each index for all the index in their respective vector.

Space complexity:
Undirected: O(N+2E)
Directed: O(N+E)
If there is an edge weight as well, then we can convert the vector into pair <int, int> instead of int.
Connected components in a graph:
A graph can either be a connected graph, or disconnected graph.
In general, whatever code we write, we have to write for multiple components.
Traversal Techniques:
Breadth-First Search (BFS):

BFS is traversing the adjacent nodes at first, and after that moving on to next nodes.
At first we have to take a visited array, and every thing will be marked as zero which means none of the nodes has been visited yet.
Run a for loop so that it calls bfs for every component of the graph.
Now, we take a queue, insert the first node into it, and mark it as visited in the visited array.
Now, iterate till the queue is not empty.
Take the topmost element of the queue, and remove that element from the queue.
Take the adjacent nodes of the node that we popped from the queue, and insert those nodes into the queue (only if not visited). Mark visited for those nodes as well.
Continue this till we get all the adjacent nodes, for all nodes. (The queue is empty).
Complexity:
Time complexity: O(N+E) -> N is time taken for visiting N nodes, and E is for travelling through adjacent nodes overall.
Space complexity: O(N+E) + O(N) + O(N) -> Space for adjacency list, visited array, and queue.
Depth-First Search:

DFS is traversing the graph in depthward motion. It is a recursive function.
Create a visited array of size V+1. Mark all the indexes as zero.
Run a for loop so that it calls bfs for every component of the graph.
Take the first node, mark it as visited.
Make a recursive call to its adjacent nodes, and mark those nodes as visited.
Continue this recurvise call till there is no further recursive dfs calls that will be made.
Come back, and make recursive calls for other adjacent nodes.
Complexity:
Time complexity: O(N+E) -> N is time taken for visiting N nodes, and E is for travelling through adjacent nodes overall.
Space complexity: O(N+E) + O(N) + O(N) -> Space for adjacency list, visited array, and auxiliary space.
Cycle detection in undirected graph:
Usign BFS:

If any of the adjacent node has been visited previously, then we can say that there is a cycle.
Create a visited array of size V+1. Mark all the indexes as zero.
Run a for loop so that it calls bfs for every component of the graph.
In the normal bfs we did just put the node in the queue, but here we will modify it.
Here we are going to put the node, as well as their previous/parent node in the queue.
Take the previous/parent node for the starting node as -1.
Mark the node as visited after putting it into the queue.
Take every node from the queue and check if it's already been visited (not including the parent node).
If we get any node that's already been visited, then we can return true (cycle is present), else cycle is not present.
Complexity:
Time complexity: O(N+E) -> N is time taken for visiting N nodes, and E is for travelling through adjacent nodes overall.
Space complexity: O(N+E) + O(N) + O(N) -> Space for adjacency list, visited array, and queue.
Using DFS:

While traversing in a depth order fashion, if any of the node has been visited previously, then we can say that there is a cycle.
Create a visited array of size V+1. Mark all the indexes as zero.
Run a for loop so that it calls bfs for every component of the graph.
In the normal dfs we did just take the node and do a recursive dfs call.
Here we are going to take the node, as well as their previous/parent node for making recursive call.
Take the previous/parent node for the starting node as -1.
Make a recursive call to its adjacent nodes, and mark those nodes as visited.
If we get any adjacent node, that's already been visited (not including the parent node), return true (cycle present), else if there are no further recursion calls, return false (cycle not present).
Bipartite graphs:
A graph that can be coloured using exactly two colors such that no two adjacent nodes have same color.
Observation:
Any graph that has an odd length cycle cannot be a bipartite.
Any graph that doesn't have an odd length cycle can be a bipartite graph (graph with even length cycle, or graph with no cycle).
Using BFS:
Create a color array of size V+1. Mark all the indexes as -1. Take two colours (0 and 1).
Now, we take a queue, insert the first node into it, and color it with color 1.
Take the adjacent node, put that node in the queue, and color it with exact opposite color (0).
Continue this for further adjacent nodes.
Also keep checking the color of adjacent nodes for each node.
If we come across a situation where two adjacent nodes have same color, we will return false (graph is not bipartite), else return true.
Complexity:
Time complexity: O(N+E) -> N is time taken for visiting N nodes, and E is for travelling through adjacent nodes overall.
Space complexity: O(N+E) + O(N) + O(N) -> Space for adjacency list, visited array, and queue.
Using DFS:
Create a color array of size V+1. Mark all the indexes as -1. Take two colours (0 and 1).
Mark the color for the first node (1) and do a recursive call for the adjacent node.
Mark the adjacent node with exactly opposite color, and continue the recursive call for the next adjacent nodes.
Also keep checking the color of adjacent nodes for each node.
If we come across a situation where two adjacent nodes have same color, we will return false (graph is not bipartite), else return true.
Complexity:
Time complexity: O(N+E) -> N is time taken for visiting N nodes, and E is for travelling through adjacent nodes overall.
Space complexity: O(N+E) + O(N) + O(N) -> Space for adjacency list, visited array, and auxiliary space.
Cycle detection in directed graph:
Using DFS:

We will have to use a self dfs along with visited dfs in case of directed graph.
We need to create two visited arrays.
Visited, and
Dfs visited. (To keep a track if a node was visited in the same dfs call or not).
We take the initial value of node and check if that is unvisited. If that is unvisited we call the cycleCheck for that node.
Whenever cycleCheck of a node is called, we mark the visited and dfs visited array for that node as visited.
Now we call the cycleCheck for the adjacent node. (Also, mark visited in both arrays).
We do recurvise calls on the adjacent and next adjacent nodes.
Once we reach to a point where there is no further adjacent nodes that's not visited, there will be no further recursion calls.
We will try to go back.
Whenever a recursion call for a function or a dfs is over, we will take that value out of dfs visited array. (Because that dfs is over for that node).
If while traversing, we find a node that has been visited in both the arrays, then only we can say that cycle is present in the graph.
Complexity:
Time complexity: O(N+E) -> N is time taken for visiting N nodes, and E is for travelling through adjacent nodes overall.
Space complexity: O(N+E) + O(2N) + O(N) -> Space for adjacency list, visited and dfs visited array, and auxiliary space.
Using BFS (Kahn's Algorithm):

Kahn's algorithm is finding topological sort, and topological sort is only possible for directed acylic graph.
So, if a graph has cycle, then topological sort won't be possible.
Here we will use the reverse logic of topological sort.
We will try to generate topological sort, and if we are unable to generate then we can conclude that it's a cyclic graph.
Topological Sorting:
It is defined as a linear ordering of vertices such that if there is an edge u->v, then u appears before v in that ordering.
There can be multiple topological sorting for a given graph.
It is possible only for directed acylic graph.
Using DFS:
We need to run a for loop for all the nodes.
If any node in the for loop is unvisited, then we will call the dfs for that node.
We also need to carry a data strucuture (stack), and a visited array (initially marked 0).
Whenever all the adjacent nodes have been visited for the current node, (that means dfs for that node is over), then we will put that node into the stack.
Once the for loop is over, the stack will be the desired output of topological sort.
Complexity:
Time complexity: O(N+E) -> N is time taken for visiting N nodes, and E is for travelling through adjacent nodes overall.
Space complexity: O(N+E) + O(2N) + O(N) -> Space for adjacency list, visited array and stack, and auxiliary space.
Using BFS (Kahn's Algorithm) :
BFS algorithm will be requiring two things.
Indegree array, marked 0 at the beginning. (Indegree: Number of incoming edges).
A queue.
Find out indegree for all the nodes.
Traverse through the indegree array, and push the nodes with 0 indegrees in the queue.
Apply bfs for the elements in queue.
For each element in the queue, check out the adjacent nodes.
Print that element.
Go to the adjacent nodes of the element taken in the indegree array, and reduce the indegree for those nodes by 1.
Anytime the indegree of any node becomes 0, we will push that node in the queue.
Continue this till the queue gets empty.
Complexity:
Time complexity: O(N+E) -> N is time taken for visiting N nodes, and E is for travelling through adjacent nodes overall.
Space complexity: O(N+E) + O(N) + O(N) -> Space for adjacency list, indegree array, and queue.
Shortest Path in Undirected Graph with Unit Weights:
We need to find the shortest distance to every node, from the given source.
We can solve this using bfs and doing some modifications to it.
We will take a distance array of size similar to the number of nodes, and mark each as infinity.
Now we will take a queue that is always going to store the nodes.
Put the source node into the queue, and mark its distance as zero.
Take the source node out, and check its adjacent nodes.
Take the adjacent nodes of the node and mark their distace as 1.
Put those adjacent nodes into the queue.
Take the nodes from queue one by one.
For each node check their adjacent nodes.
For each adjacent node, to get the distance of that adjacent node add a distance of 1 to the distance of the node from source node, and also compare it to the already assigned distance. Take the minimum of both.
Conitinue this till the queue gets empty.
Complexity:
Time complexity: O(N+E) -> N is time taken for visiting N nodes, and E is for travelling through adjacent nodes overall.
Space complexity: O(N+E) + O(N) + O(N) -> Space for adjacency list, distance array, and queue.
Shortest Path in weighted Directed Acyclic Graph:
Given a source, we need to find the shortest distance for every other node from source.
We will find the topological sort and then it will be a simple comparison algorithm on bfs.
Since it's a weighted graph, we need to store the node in pairs (node and weight).
We will need a stack, a visited array, and a distance array.
Mark every node as unvisited in visited array.
Mark every node as infinity in distance array.
Take the source node, mark it as visited.
Do a recursion call to its adjacent nodes one by one for all the adjacent nodes. There will be no calls made for the node that's already been visited.
Keep marking the nodes as visited whenver a call is being made to that node.
When we reach to a point where there is no further recursion calls, take the end node(the node from which we are returning back, similar thing we did in dfs topological sort) and put it into the stack.
We will get the topological sort in the stack.
Make the distance of source element as zero in the distance array.
Take the first element out of the stack.
Check the distance of first element (it should not be infinity).
Take the adjacent nodes one by one for that element and add the distance of that adjacent node to the distance of the element, and also compare it to the already assigned distance.
Take the minimum of both.
Continue this for all the nodes (in the stack) till the stack gets empty.
Complexity:
Time complexity: O(2(N+E)) -> N is time taken for visiting N nodes, and E is for travelling through adjacent nodes overall. We are traversing twice.
Space complexity: O(N+E) + O(N) + O(N) + O(N/0) -> Space for stack, distance array, and (auxiliary space, depends if we are using dfs or bfs for topological sort).
Shortest Path in Undirected Graphs (Dijkstra's Algorithm):
We will have a priority queue (that will store distance, node).
Priority queue should be a min heap, so that the element with the lower distance should always be on top of priority queue.
We will also be having a distance array of the size of nodes+1, initially marked as infinity.
Take the first node out of queue and mark its distance as zero.
Insert the distance of that node, and the node in the priority queue.
Now start iterating the graph similar to bfs.
Take the adjacent nodes of that element one by one, and add the distance of that adjacent node to the distance of the element, and also compare it to the already assigned distance.
Put the node and distance in the priority queue.
Take the first element from the priority queue. (It will be storing the element with lower distant on top).
Take the adjacent nodes, and repeat the above process. (7,8,9)
Continue this till the priority queue becomes empty.
Complexity:
Time complexity: O((N+E)logN) -> N is time taken for visiting N nodes, and E is for travelling through adjacent nodes overall. Log N is for priority queue.
Space complexity: O(N+E) + O(N) + O(N)
Minimum spanning tree:
Whenever we can draw a tree from a given graph such that the tree has all the N nodes, and the number of edges is N-1 such that every node can be reachable from every other node, that's when we call it a minimum spanning tree.
The minimum refers to the spanning tree with the minimum weight amongst all the other spanning trees that can be formed from a graph.
This can be find using two different algorithms:
Prim's Algorithm
Kruskal's Algorithm
Prim's Algorithm for Minimum Spanning Tree:
Intuition:
Start with the first node, and find the minimum weighted edge attached to this node.
Take that edge, and the node connected to that edge.
Check out all the adjacent edges among both the nodes which one is the minimal.
Take the minimal edge along with the node.
Now continue the same for all the three nodes. Find the adjacent node with the minimal edge weight.
Do this until all the nodes are covered.
The tree that we will get would be the MST.
Implementation (Brute Force):
We will be requiring three different arrays.
Key array of size N (everything initialized to infinity in the beginning apart from the zero index).
MST array (everything initialized to false in the beginning).
Parent array (everythin initialized to -1 in the beginning).
Find out the node with the minimum possible key value that is not the part of the mst and take that node.
Mark the node as true in mst array.
Check out all the adjacent nodes of that node.
For each adjacent node of that node, mark the edge weight for that adjacent node as the weight in the key array, and also mark its parent in the parent array.
Take the next node from the key array with minimum value and not part of mst.
Mark that node as true in the mst array.
Again check for all the adjacent nodes (5,6,7), but don't consider the nodes that are marked true in mst array.
If at any instance the weight of any adjacent node is already having a lesser weight, then don't change the weight for that node.
Continue this till all the nodes in mst becomes true.
Iterate the parent array and form the tree.
The tree that we will get would be the MST.
Complexity:
Time complexity: O(>N^2)
Space complexity: O(N+E) + O(3N)
Implementation (Efficient approach):
We can use priority queue that will give us the minimal value from the key array.
Do the modifications accordingly.
Complexity:
Time complexity: O(N+E+NlogN)
Space complexity: O(N+E) + O(4N)
Kruskal's Algorithm (Using Disjoint Set):
We are not going to store the graph in adjacency list. We will store it in a simple linear data strucutre.
Sort all the edges according to weight.
Greedily pick up the shortest edge.
Check if both the nodes of that edge belong to same component or not (using disjoint set).
If not take that edge.
Next, again greedily pick up the next shortest edge. Then 4,5.
If there comes a case where two nodes of an edge belongs to the same component we will not take that edge.
Do this for all the edges.
Once all the edges are covered, the resulting tree will be our minimum spanning tree.
Complexity: (for M edges)
Time complexity: O(MlogM) + O(M*4α) ≈ O(MlogM) -> Sort the edges, iterating over the edges and checking if they belong to same component or not.
Space complexity: O(3M) + O(N) + O(N) -> For storing edges, parent and rank array for implementation of disjoint set.
Disjoint Set | Union By Rank and Path Compression:
Disjoint Set data strucuture:
This data structure is going to provide us with two different operations:
findParent()
Union()
Assume there are 7 nodes and each node belongs to different component.
Every one is its parent itself.
Now, if we need to find the parent of any node then we will get the same node only.
Now, Union(1,2)-> which means combine them in a single component.
After combining, let's assume one node as the parent for both.
Now, do Union(2,3)-> hence 3 nodes in a single component.
Again, let's assume 1 as the parent of all three.
Now, Union(4,5) -> Parent:4.
Now, Union(6,7) -> Parent:6.
Now, Union(5,6) -> Parent:4.
If two nodes has same parent, then they belong to one component.
Now, Union(3,7) -> Parent:1.
So, the efficient implementation of disjoint set is done by union by rank, and path compression.
So, we make sure that there is a parent array, and every node has itself as the parent when we start.
We also maintain another array (rank array), which is going to store the rank of all these nodes.
Make initial rank of all the nodes as zero.
Start doing the union operation.
For the first two nodes, make any of them as the parent (since rank for both the nodes are zero).
Whenever we are attaching two similar rank nodes, we need to make sure that the node to whom we are attaching, we need to increase it's rank by 1.
Take the next union, compare ranks, and the one with higher rank will be the parent for that node.
Do path compression whenever required.
Complexity:
Time complexity: O(4α) ≈ O(4) -> If there are m operations we are doing m time as every union operation is done in constant time (4α is mathematically proved).
Space complexity: O(2N) -> Rank array, and parent array.
Bridges in Graph || Cut Edge:
Bridge are those edges in a graph on whose removal the graph is broken into two or more number of components.
In order to find out the bridges in a graph:
We will be using two different arrays:
Time of insertion for a given node.
Lowest time of insertion.
Start with the first node(1). Mark it's time of insertion as 1 and lowest time of insertion also as 1.
Move to the next node(2). Mark it's time of insertion as 2 and lowest time of insertion also as 2.
Do this for all the nodes in the graph in a dfs manner.
If while returning at any moment we find any node having less lowest time of insertion in the from from which it is returning as compared to itself, it marks its lowest time of insertion same as that of its adjacent node. (We are only updating the value low whenever the dfs for the adjacent node is completed).
If low[adjacent] > time of insertion[node] then we can say it's a bridge.
Complexity:
Time complexity: O(N+E) -> We are doing a dfs only.
Space complexity: O(N+E) + O(2N) and auxiliary space.
Articulation Point | Cut Vertex:
Articulation point is a node on whose removal the graph is broken down into two or more number of components.
We will be using two different arrays:
Time of insertion for a given node.
Lowest time of insertion.
Start with the first node(1). Mark it's time of insertion as 1 and lowest time of insertion also as 1.
Move to the next node(2). Mark it's time of insertion as 2 and lowest time of insertion also as 2.
Do this for all the nodes in the graph in a dfs manner.
If while returning at any moment we find any node having less lowest time of insertion in the from from which it is returning as compared to itself, it marks its lowest time of insertion same as that of its adjacent node. (We are only updating the value low whenever the dfs for the adjacent node is completed).
If low[it] >= time of insertion[node] && parent != -1, then we can say the node is an articulation point.
The starting parent can only be an articulation point if ( individual childs > 1 && parent == -1).
Note: These articulation points might repeat while traversing.
Complexity:
Time complexity: O(N+E) -> dfs function.
Space complexity: O(N+E) +O(2N) + O(N) -> 2 arrays, amd auxiliary space.
Kosaraju's Algorithm for Strongly Connected Components (SCC):
This algorithm helps us to find all the strongly connected components in a directed graph.
Strongly connected components: A component in which if you start from any node, you can reach every other node in that component.
Kosaraju's algorithm tries to have a dfs right starting from the back edges so that only those nodes in the scc are visited.
Sort all the nodes in order of finishing time. (Apply topological sort).
Transpose the graph. (Reverse all the edgees).
Do the dfs according to the finishing time.
Complexity:
Time complexity: O(N) + O(N+E) + O(N+E) -> Topological sort, Transpose, DFS.
Space complexity: O(N+E) + O(N+E) + O(N) + O(N) -> Transpose graph, visited array, stack.
Bellman Ford Algorithm (Detect Negative Weight Cycle in Graphs):
Dijkstra's algorithm does give us the shortest path from source to every other node, but dijkstra's algorithm works for only positive weighted edge.
If we follow the Dijkstra's algorithm, and there is an edge with negative weigth then it will end up in an infinite loop.
Bellman Ford will work for negative edges in directed graph, but there is a certain condition.
If there is a negative cycle, then we will not be able to find out the shortest parth.
Bellman Ford will tell us if there is a negative cycle or not.
Also, if there is an undirected graph, then we will first convert the graph into a directed graph using bidirectional edges.
We need to take a distance array.
For source mark the distance as zero, and for every other node mark it as infinity.
Relax all the edges (N-1) times.
if(dist[u]+wt <dist[v]) dist[v]= dist[u]+wt;
The distances we get after relaxing (N-1) times is the shortest distance.
To detect if it's a negative edge weight:
Relax the edges one more time.
If distance reduces further, that means it has a negative cycle.
Complexity:
Time complexity: O(N-1) * O(E) -> Very bad as compared to Dijkstra's.
Space complexity: O(N+E) + O(N) ->distance array.
Floyd Warshall Algorithm
Tarjan's Algorithm
