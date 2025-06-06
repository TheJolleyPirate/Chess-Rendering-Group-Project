#pragma once
#include <object.hpp>
#include <cfloat>

void checkMesh(Object &object, const std::string &fileName);
bool saveMeshASOBJ(const Object &object);

class Node;
class Trapezoid;
class SeidelRay;

/*by Daniel Jolley-Rogers u7511912
defines a node for the Seidel tree*/
class Node{
    public:
        enum NodeType{
            ROOT,
            VERTEX,
            EDGE,
            TRAPAZOID,
        };
        enum NodeType nodeType;
        std::weak_ptr<Vertex> vertex; //the vertex this represents if nodetype is vertex
        std::shared_ptr<HalfEdge> edge; //the edge this represents if nodetype is edge
        std::shared_ptr<Trapezoid> trapezoid; //the trapezoid this represents if nodetype is trapezoid
        const int id; //unique node id
        std::shared_ptr<Node> left; //left child node
        std::shared_ptr<Node> right; //right child node
        std::weak_ptr<Node> parent; //parent node
        Node(const int _id): id(_id){}
};

/*by Daniel Jolley-Rogers u7511912
defines a trapezoid for the Seidel tree*/
class Trapezoid{
    public:
        std::vector<std::shared_ptr<Trapezoid>> up; //trapezoids directly above this one
        std::vector<std::shared_ptr<Trapezoid>> down; //trapezoids directly below this one
        std::shared_ptr<HalfEdge> lseg; //left edge
        std::shared_ptr<HalfEdge> rseg; //right edge
        std::weak_ptr<Node> sink; //Position of trapezoid in tree structure
        bool validState; //Represents validity of trapezoid (Inside or outside)
        float highRay; //the ray which bounds the top of the trapezoid
        float lowRay; //the ray which bounds the bottom of the trapezoid
        Trapezoid(std::weak_ptr<Node> weakNode, float lowRay, float highRay){
            this->sink = weakNode.lock()->parent;
            this->highRay = highRay;
            this->lowRay = lowRay;
            setEdges(weakNode);
        }
        void setEdges(const std::weak_ptr<Node> &weakNode){
            std::shared_ptr<Node> node = weakNode.lock();
            int previous = node->id;
            std::shared_ptr<Node> current = node->parent.lock();
            if(!current){
                return;
            }
            bool foundLeft = false;
            bool foundRight = false;
            while(true){
                if(current->nodeType == Node::EDGE){
                    if (!foundLeft && (!current->left  || current->left->id  != previous)) { 
                        lseg = current->edge;
                        foundLeft = true;
                        if(foundRight){
                            return;
                        }
                    }
                    if (!foundRight && (!current->right  || current->right->id  != previous)) { 
                        rseg = current->edge;
                        foundRight = true;
                        if(foundLeft){
                            return;
                        }
                    }
                }
                previous = current->id;
                current = current->parent.lock();
                //if reached the top of the tree
                if(!current){
                    return;
                }
            };
        }
        
        bool inPolygon(){
            validState = false;
            if(!lseg || !rseg || !highRay || !lowRay){
                return false;
            }
            
            //get current node from trapezoid
            std::shared_ptr<Node> currentNode = sink.lock();
            //traverse upward until left or right edge found
            bool directionLeft;
            while(true){
                int previous = currentNode->id;
                currentNode = currentNode->parent.lock();
                if(currentNode == nullptr){
                    return false;
                }
                if(currentNode->nodeType == Node::EDGE){
                    if(currentNode->right != nullptr && currentNode->right->id == previous){
                        directionLeft = true;
                    }
                    else{
                        directionLeft = false;
                    }
                    break;
                }
            }

            int counterRight = 0;
            findAdjacentTrapezoid(currentNode, directionLeft, counterRight);

            // try the opposite direction as a backup
            int counterLeft = 0;
            findAdjacentTrapezoid(currentNode, !directionLeft, counterLeft);

            if ((counterRight % 2) || (counterLeft % 2)) {
                validState = true;
                return true;
            }
            else {
                validState = false;
                return false;
            }
        }
        void setValidState(){
            inPolygon();
        }
    private:
        void findAdjacentTrapezoid(std::shared_ptr<Node> currentNode, bool directionLeft, int &counter){
            bool subDirectionLeft = true;
            //get node according to direction
            if(directionLeft){
                currentNode = currentNode->left;
            }
            else{
                currentNode = currentNode->right;
            }
            //find the closest trapezoid from that node
            while(true){
                if(currentNode->nodeType == Node::TRAPAZOID){
                    counter += 1;
                    //check if tree outside polygon
                    if(currentNode->trapezoid->lseg && currentNode->trapezoid->rseg){
                        //get segment according to direction
                        while(true){
                            int previous = currentNode->id;
                            currentNode = currentNode->parent.lock();
                            if(!currentNode){
                                return;
                            }
                            if(currentNode->nodeType == Node::EDGE){
                                if(directionLeft && currentNode->right && currentNode->right->id == previous){
                                    break;
                                }
                                else if(!directionLeft && currentNode->left && currentNode->left->id == previous){
                                    break;
                                }
                            }
                        }
                        findAdjacentTrapezoid(currentNode, directionLeft, counter);
                    }
                    break;
                }
                //we are trying to move just one spot in direction
                else if(currentNode->nodeType == Node::EDGE){
                    if(directionLeft){
                        currentNode = currentNode->right;
                    }
                    else{
                        currentNode = currentNode->left;
                    }
                }
                //this works due to the fact that higher nodes are added first and are thus higher in the tree
                else if(currentNode->nodeType == Node::VERTEX){
                    if(subDirectionLeft){
                        currentNode = currentNode->left;
                        subDirectionLeft = false;
                    }
                    else{
                        currentNode = currentNode->right;
                        subDirectionLeft = true;
                    }
                }
            }
        }
};
