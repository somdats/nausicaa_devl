#pragma once
#include <vcg/complex/complex.h>

class CFace;
class CVertex;
struct MyUsedTypes : public vcg::UsedTypes<	vcg::Use<CVertex>		::AsVertexType,
    vcg::Use<CFace>			::AsFaceType> {};

/// compositing wanted proprieties
class CVertex : public vcg::Vertex< MyUsedTypes, vcg::vertex::Coord3f, vcg::vertex::Normal3f, vcg::vertex::BitFlags> {};
class CFace : public vcg::Face<  MyUsedTypes, vcg::face::VertexRef, vcg::face::Normal3f, vcg::face::BitFlags > {};
class CMesh : public vcg::tri::TriMesh< std::vector<CVertex>, std::vector<CFace> > {};
