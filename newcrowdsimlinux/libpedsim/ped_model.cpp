//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//
//
// Adapted for Low Level Parallel Programming 2015
//

#include "omp.h"
#include "pthread.h"
#define NTHREADS 16
#define True false
#include "ped_model.h"
#include "ped_waypoint.h"
#include "cuda_dummy.h"
#include "ped_model.h"
#include <iostream>
#include <stack>
#include <algorithm>
#include <emmintrin.h>
#include <math.h>

void Ped::Model::setup(std::vector<Ped::Tagent*> agentsInScenario)
{
  agents = std::vector<Ped::Tagent*>(agentsInScenario.begin(), agentsInScenario.end());
  treehash = new std::map<const Ped::Tagent*, Ped::Ttree*>();

  // Create a new quadtree containing all agents
  tree = new Ttree(NULL, treehash, 0, treeDepth, 0, 0, 1000, 800);
  for (std::vector<Ped::Tagent*>::iterator it = agents.begin(); it != agents.end(); ++it)
  {
    tree->addAgent(*it);
  }

  // This is the sequential implementation
  implementation = VECTOR;//OMP;

  // Set up heatmap (relevant for Assignment 4)
  setupHeatmapSeq();
}

typedef struct segment_{
  int start;
  int end;
  std::vector<Ped::Tagent*> tagent;
} segment;

void *computepos(void *ptr){
  segment *seg = (segment*)ptr;
  for( int i = seg->start; i < seg->end; i++){
    seg->tagent[i]->computeNextDesiredPosition();
  }
  return NULL;
}
void *movepos(void *ptr){
  segment *seg = (segment*)ptr;
  for( int i = seg->start; i < seg->end; i++){
    seg->tagent[i]->setX(seg->tagent[i]->getDesiredX());
    seg->tagent[i]->setY(seg->tagent[i]->getDesiredY());
  }
  return NULL;
}


void Ped::Model::thisIsAFunction(){
  /***************Add SIMD things here***************/
  std::vector<Ped::Tagent*> tagent = getAgents();
  unsigned int tagent_size= tagent.size();
  
  unsigned int vector_size = tagent_size + 4 - (tagent_size % 4);
  float destinationGetX[vector_size] __attribute__((aligned (16)));
  float destinationGetY[vector_size] __attribute__((aligned (16)));
  float x[vector_size] __attribute__((aligned (16)));
  float y[vector_size] __attribute__((aligned (16)));
  float destinationGetR[vector_size] __attribute__((aligned (16)));
  
  #pragma omp parallel for 
  for(int i=0; i < vector_size; i++){
    destinationGetX[i] = tagent[i%tagent_size]->destination == NULL ? 1 : tagent[i%tagent_size]->destination->getx();
    destinationGetY[i] = tagent[i%tagent_size]->destination == NULL ? 1 : tagent[i%tagent_size]->destination->gety();
    destinationGetR[i] = tagent[i%tagent_size]->destination == NULL ? 1 : tagent[i%tagent_size]->destination->getr();
    x[i] = tagent[i%tagent_size]->getX();
    y[i] = tagent[i%tagent_size]->getY();
  }
  
  for(int i=0; i < tagent_size; i++){
    cout<<"X: "<<x[i]<<endl;
  }
  
  //float diffX[vector_size]__attribute__((aligned (16)));
  //float diffY[vector_size]__attribute__((aligned (16)));
  
  float agentReachedDestination[vector_size] __attribute__((aligned (16)));

  //#pragma omp parallel for 
  for(int i=0; i < vector_size; i+=4){
    // compute if agent reached its current destination
    __m128 pSrcX = _mm_load_ps(x + i);
    __m128 pSrcY = _mm_load_ps(y + i);
    __m128 getR = _mm_load_ps(destinationGetR + i);
    __m128 pSrcDestx= _mm_load_ps(destinationGetX + i);
    __m128 pSrcDesty= _mm_load_ps(destinationGetY + i);
    __m128 diffX = _mm_sub_ps(pSrcDestx,pSrcX);
    __m128 diffY = _mm_sub_ps(pSrcDesty,pSrcY);
    //__m128 diffX = _mm_add_ps((__m128*)(destinationGetX+i),(__m128*)(x+i));
    //__m128 diffY = _mm_add_ps((__m128*)(destinationGetY+i),(__m128*)(y+i));
    //double diffX = destination->getx() - x;
    //double diffY = destination->gety() - y;
    __m128 mul1 = _mm_mul_ps(diffX, diffX);
    __m128 mul2 = _mm_mul_ps(diffY, diffY);
    __m128 mulsum = _mm_add_ps(mul1, mul2);
    __m128 length = _mm_sqrt_ps(mulsum); 
    
    //double length = sqrt(diffX * diffX + diffY * diffY);
    __m128 cmplt = _mm_cmplt_ps(length,getR);
    
    _mm_store_ps(agentReachedDestination+i, cmplt);
      //agentReachedDestination = length < destination->getr();
  }
  
  
  //#pragma omp parallel for 
  for(int i=0; i < tagent_size; i+=1){
  if(agentReachedDestination[i]<0){
  cout << "true" << endl; }
  else{
  //  cout << agentReachedDestination[i] << endl;
  //cout << "false" << endl; 
}
  if (((fabs(agentReachedDestination[i])<0.001 && tagent[i]->destination != NULL) || tagent[i]->destination == NULL) && !tagent[i]->waypoints.empty()) {
    // Case 1: agent has reached destination (or has no current destination);
    // get next destination if available
    tagent[i]->destination = tagent[i]->waypoints.front(); tagent[i]->waypoints.pop_front(); // pop
    
  }
  else {
    // else else else else else else else else else else else else else else else else else else else 
    // Case 2: agent has not yet reached destination, continue to move towards
    // current destination
    // destination == destination (y)
  }
  }
  float notRoundedDesiredPositionXNeedToBeRoundedBeforeSettingDesiredPositionX[vector_size]__attribute__((aligned (16)));
  float notRoundedDesiredPositionYNeedToBeRoundedBeforeSettingDesiredPositionY[vector_size]__attribute__((aligned (16)));   // compute if agent reached its current destination
    
for(int i=0; i < vector_size; i+=4){
  
  __m128 pSrcX = _mm_load_ps(x + i);
  __m128 pSrcY = _mm_load_ps(y + i);
  __m128 pSrcDestx= _mm_load_ps(destinationGetX + i);
  __m128 pSrcDesty= _mm_load_ps(destinationGetY + i);
  __m128 diffX = _mm_sub_ps(pSrcDestx,pSrcX);
  __m128 diffY = _mm_sub_ps(pSrcDesty,pSrcY);
  //__m128 diffX = _mm_add_ps((__m128*)(destinationGetX+i),(__m128*)(x+i));
  //__m128 diffY = _mm_add_ps((__m128*)(destinationGetY+i),(__m128*)(y+i));
  __m128 mul1 = _mm_mul_ps(diffX, diffX);
  __m128 mul2 = _mm_mul_ps(diffY, diffY);
  __m128 mulsum = _mm_add_ps(mul1, mul2);
  __m128 length = _mm_sqrt_ps(mulsum); 
  
  __m128 divXLength = _mm_div_ps(diffX, length);    
  __m128 divYLength = _mm_div_ps(diffY, length);
  __m128 resultDesiredX = _mm_add_ps(divXLength, pSrcX); 
  __m128 resultDesiredY = _mm_add_ps(divYLength, pSrcY); 
  _mm_store_ps(notRoundedDesiredPositionXNeedToBeRoundedBeforeSettingDesiredPositionX,resultDesiredX);       
  _mm_store_ps(notRoundedDesiredPositionYNeedToBeRoundedBeforeSettingDesiredPositionY,resultDesiredY);
 }
 
#pragma omp parallel for 
for(int i=0; i < tagent_size; i+=1){
  if (tagent[i]->destination != NULL ) {
    tagent[i]->setDesiredPosition(round(notRoundedDesiredPositionXNeedToBeRoundedBeforeSettingDesiredPositionX[i]),round(notRoundedDesiredPositionYNeedToBeRoundedBeforeSettingDesiredPositionY[i]));
  }
 }
 
 return;
}


void Ped::Model::tick()
{
  std::vector<Ped::Tagent*> tagent = getAgents();
// EDIT HERE FOR ASSIGNMENT 1
  pthread_t pids[NTHREADS];
  segment segs[NTHREADS];

 switch(implementation){
    case VECTOR:
      thisIsAFunction();
      //#pragma omp parallel for
      for(int i = 0; i < tagent.size(); i++){
  cout << "(" << tagent[i]->getDesiredX() << ", " << tagent[i]->getDesiredY() << ")\n";
	tagent[i]->setX(tagent[i]->getDesiredX());
	tagent[i]->setY(tagent[i]->getDesiredY());
}
	break;

    case OMP:
      omp_set_num_threads(NTHREADS);
      #pragma omp parallel for
      for(int i = 0; i < tagent.size(); i++){
        tagent[i]->computeNextDesiredPosition();
      }

      #pragma omp parallel for
      for(int i = 0; i < tagent.size(); i++){
        tagent[i]->setX(tagent[i]->getDesiredX());
        tagent[i]->setY(tagent[i]->getDesiredY());
      }
      break;

    case PTHREAD:
      for(int i = 0; i < NTHREADS; i++){
        segs[i].start=i*tagent.size()/NTHREADS;
        segs[i].end= (i+1)*tagent.size()/NTHREADS;
        segs[i].tagent = tagent;
      }
      if(segs[NTHREADS-1].end != tagent.size()){
	cerr << "(segs[NTHREADS-1].end != tagent.size())" << endl;
      }

      for(int i = 0; i < NTHREADS; i++){
        pthread_create( &pids[i], NULL, &computepos, &(segs[i]));
      }
      
      for(int i = 0; i < NTHREADS; i++){
        pthread_join( pids[i], NULL);
      }

      for(int i = 0; i < NTHREADS; i++){
        pthread_create( &pids[i], NULL, &movepos, &(segs[i]));
      }
      
      for(int i = 0; i < NTHREADS; i++){
        pthread_join( pids[i], NULL);
      }

      break;

    case SEQ:
      for(int i = 0; i < tagent.size(); i++){
        tagent[i]->computeNextDesiredPosition();
      }
  
      for(int i = 0; i < tagent.size(); i++){
        tagent[i]->setX(tagent[i]->getDesiredX());
        tagent[i]->setY(tagent[i]->getDesiredY());
      }
      break;

    default:
      break;
  }

}

////////////
/// Everything below here relevant for Assignment 3.
/// Don't use this for Assignment 1!
///////////////////////////////////////////////
void Ped::Model::move( Ped::Tagent *agent)
{
  // Search for neighboring agents
  set<const Ped::Tagent *> neighbors = getNeighbors(agent->getX(), agent->getY(), 2);
    
  // Retrieve their positions
  std::vector<std::pair<int, int> > takenPositions;
  for (std::set<const Ped::Tagent*>::iterator neighborIt = neighbors.begin(); neighborIt != neighbors.end(); ++neighborIt) {
    std::pair<int,int> position((*neighborIt)->getX(), (*neighborIt)->getY());
    takenPositions.push_back(position);
  }

  // Compute the three alternative positions that would bring the agent
  // closer to his desiredPosition, starting with the desiredPosition itself
  std::vector<std::pair<int, int> > prioritizedAlternatives;
  std::pair<int, int> pDesired(agent->getDesiredX(), agent->getDesiredY());
  prioritizedAlternatives.push_back(pDesired);

  int diffX = pDesired.first - agent->getX();
  int diffY = pDesired.second - agent->getY();
  std::pair<int, int> p1, p2;
  if (diffX == 0 || diffY == 0)
  {
    // Agent wants to walk straight to North, South, West or East
    p1 = std::make_pair(pDesired.first + diffY, pDesired.second + diffX);
    p2 = std::make_pair(pDesired.first - diffY, pDesired.second - diffX);
  }
  else {
    // Agent wants to walk diagonally
    p1 = std::make_pair(pDesired.first, agent->getY());
    p2 = std::make_pair(agent->getX(), pDesired.second);
  }
  prioritizedAlternatives.push_back(p1);
  prioritizedAlternatives.push_back(p2);

  // Find the first empty alternative position
  for (std::vector<pair<int, int> >::iterator it = prioritizedAlternatives.begin(); it != prioritizedAlternatives.end(); ++it) {

    // If the current position is not yet taken by any neighbor
    if (std::find(takenPositions.begin(), takenPositions.end(), *it) == takenPositions.end()) {

      // Set the agent's position 
      agent->setX((*it).first);
      agent->setY((*it).second);

      // Update the quadtree
      (*treehash)[agent]->moveAgent(agent);
      break;
    }
  }
}

/// Returns the list of neighbors within dist of the point x/y. This
/// can be the position of an agent, but it is not limited to this.
/// \date    2012-01-29
/// \return  The list of neighbors
/// \param   x the x coordinate
/// \param   y the y coordinate
/// \param   dist the distance around x/y that will be searched for agents (search field is a square in the current implementation)
set<const Ped::Tagent*> Ped::Model::getNeighbors(int x, int y, int dist) const {
  // if there is no tree, return all agents
  if(tree == NULL) 
    return set<const Ped::Tagent*>(agents.begin(), agents.end());

  // create the output list
  list<const Ped::Tagent*> neighborList;
  getNeighbors(neighborList, x, y, dist);

  // copy the neighbors to a set
  return set<const Ped::Tagent*>(neighborList.begin(), neighborList.end());
}

/// Populates the list of neighbors that can be found around x/y./// This triggers a cleanup of the tree structure. Unused leaf nodes are collected in order to
/// save memory. Ideally cleanup() is called every second, or about every 20 timestep.
/// \date    2012-01-28
void Ped::Model::cleanup() {
  if(tree != NULL)
    tree->cut();
}

/// \date    2012-01-29
/// \param   the list to populate
/// \param   x the x coordinate
/// \param   y the y coordinate
/// \param   dist the distance around x/y that will be searched for agents (search field is a square in the current implementation)
void Ped::Model::getNeighbors(list<const Ped::Tagent*>& neighborList, int x, int y, int dist) const {
  stack<Ped::Ttree*> treestack;

  treestack.push(tree);
  while(!treestack.empty()) {
    Ped::Ttree *t = treestack.top();
    treestack.pop();
    if (t->isleaf) {
      t->getAgents(neighborList);
    }
    else {
      if (t->tree1->intersects(x, y, dist)) treestack.push(t->tree1);
      if (t->tree2->intersects(x, y, dist)) treestack.push(t->tree2);
      if (t->tree3->intersects(x, y, dist)) treestack.push(t->tree3);
      if (t->tree4->intersects(x, y, dist)) treestack.push(t->tree4);
    }
  }
}

Ped::Model::~Model()
{
  if(tree != NULL)
  {
    delete tree;
    tree = NULL;
  }
  if(treehash != NULL)
  {
    delete treehash;
    treehash = NULL;
  }
}
