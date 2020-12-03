#include "LBVH.h"
#include "../TATBasis/TATErrorReporter.h"
#include "time.h"

extern "C"
void BuildLBVH(std::vector<LBVPrim> & prims, TATVector3 * pos, std::vector<LBVNode> & leaf_nodes, std::vector<LBVNode> & internal_nodes, int num);
void LBVH::Build()
{
	clock_t begin = clock();

	BuildLBVH(m_StorePrims, &m_Positions[0], m_LeafNodes, m_InternalNodes, m_StorePrims.size());

	clock_t end = clock();
	float time = end - begin;
	TATErrorReporter::Instance()->ReportErr("build " + TString::ConvertFloat(time));


	m_Root = &m_InternalNodes[0];

	begin = clock();
	TraverseTreeB2T(); //update internal node's min max
	end = clock();
	time = end - begin;
	TATErrorReporter::Instance()->ReportErr("traverse " + TString::ConvertFloat(time));
}