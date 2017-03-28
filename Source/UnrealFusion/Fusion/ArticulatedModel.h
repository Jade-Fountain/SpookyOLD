

struct Node{
	State state;
	Ptr parent;
	Transform3D structure;

	Transform3D getGlobalPose(){
		if(cached) return cachedPose;
		return parent->getGlobalPose() * structure * state;
	}
};

class ArticulatedModel{
	std::map<NodeDescriptor,Node::Ptr> nodes;

};