#ifndef _ped_model_h_
#define _ped_model_h_
#include <vector>
#include "ped_agent.h"
#include "ped_tree.h"

#define DllExport __declspec(dllexport)

namespace Ped{
	class Ttree;
	enum IMPLEMENTATION {CUDA, VECTOR, OMP, PTHREAD, SEQ};
	class DllExport Model
	{
	public:
		void setup(std::vector<Tagent*> agentsInScenario);
		void tick();
		const std::vector<Tagent*> getAgents() const;

		// Updates the treehash, which maps each agent to the current tree node that contains it		
		void setResponsibleTree(Ped::Ttree *tree, const Ped::Tagent *agent);

		// Adds an agent to the tree structure
		void placeAgent(const Ped::Tagent *a);

		// Cleans up the tree and restructures it. Worth calling every now and then.
		void cleanup();
		~Model();

		////////////
		/// NEW
		///////////////////////////////////////////////
		int const * const * getHeatmap() const;
		int getHeatmapSize() const;
		////////////
		/// END NEW
		///////////////////////////////////////////////
	private:

		IMPLEMENTATION implementation;
		std::vector<Tagent*> agents;

		void doSafeMovement( Ped::Tagent *agent);
		// The maximum quadtree depth
		static const int treeDepth = 10;    

		// Keeps track of the positions of each agent
		Ttree *tree;

		// Maps the agent to the tree node containing it. Convenience data structure
		// in order to update the tree in case the agent moves.
		std::map<const Ped::Tagent*, Ped::Ttree*> *treehash;

		// Returns the set of neighboring agents for the specified position
		set<const Ped::Tagent*> getNeighbors(int x, int y, int dist) const;
		void getNeighbors(list<const Ped::Tagent*>& neighborList, int x, int y, int d) const;

		////////////
		/// THIS IS NEW
		///////////////////////////////////////////////
#define SIZE 1024
#define CELLSIZE 5
#define SCALED_SIZE SIZE*CELLSIZE

		int ** heatmap;

		// The scaled heatmap that fits to the view
		int ** scaled_heatmap;

		// The final heatmap: blurred and scaled to fit the view
		int ** blurred_heatmap;

		void setupHeatmapSeq();
		void updateHeatmapSeq();
		////////////
		/// END NEW
		///////////////////////////////////////////////
	};
}
#endif
