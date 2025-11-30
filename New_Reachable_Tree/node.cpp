#include "node.h"

unsigned int Node::num = 0;

/* 初始化 */
NodePool::NodePool(int n) {
	maxNode = n;
	for (int i = 0; i < n; ++i) {
		auto node = std::make_shared<Node>(this);
		buf.push(node);
	}
	printf("NodePool finished\n");
}

/* 取节点(头取法) */
ptrNode NodePool::getNode() {
	if (buf.empty()) {
		maxNode += 1000;
		for (int i = 0; i < 1000; ++i) {
			auto node = std::make_shared<Node>(this);
			buf.push(node);
		}
	}
	auto node = buf.front();
	buf.pop();
	return node;
}

/* 回收节点(置于初始状态，尾插入节点池) */
void NodePool::recycling(ptrNode node) {
	node->id_ = Node::num++;
	node->is_deadlock_ = false;
	node->is_open_ = false;
	node->discarded_ = true;
	node->g_ = 0;
	node->son_ = 0;
	node->h_ = FLT_MAX;
	node->h_pre_ = FLT_MAX;
	node->fathers.clear();
	node->sons_.clear();
	node->state_.clear();
	buf.push(node);
}