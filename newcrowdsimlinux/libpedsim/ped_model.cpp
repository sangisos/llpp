//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//
//
// Adapted for Low Level Parallel Programming 2015
//

#include "pthread.h"
#define NTHREADS 2

#include "ped_model.h"
#include "ped_waypoint.h"
#include "cuda_dummy.h"
#include "ped_model.h"
#include <iostream>
#include <stack>
#include <algorithm>

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
  implementation = PTHREAD;

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

void Ped::Model::tick()
{
  // EDIT HERE FOR ASSIGNMENT 1
  pthread_t pids[NTHREADS];
  segment segs[NTHREADS];
  std::vector<Ped::Tagent*> tagent = getAgents();

  switch(implementation){
    case OMP:
    
      #pragma OMP Parallel
      for(int i = 0; i < tagent.size(); i++){
        tagent[i]->computeNextDesiredPosition();
      }

      #pragma OMP Parallel
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
      segs[NTHREADS-1].end = tagent.size();

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
