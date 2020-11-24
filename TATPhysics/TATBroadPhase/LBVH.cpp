#include "LBVH.h"

extern "C"
void BuildLBVH(std::vector<LBVNode> & nodes, TATVector3 * pos, std::vector<LBVNode> & internal_nodes, int num);
void LBVH::Build()
{
	BuildLBVH(m_StoreNodes, &m_Positions[0], m_InternalNodes, m_StoreNodes.size());
	m_Root = &m_InternalNodes[0];

	TraverseTreeB2T(); //update internal node's min max
}