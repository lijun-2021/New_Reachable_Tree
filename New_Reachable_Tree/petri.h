#pragma once
/**
 * 可达图算法 + 搜索策略
 */

#include <queue>
#include <map>
#include <string>
#include <algorithm>
#include <ctime>
#include <mutex>
#include <list>
#include <unordered_map>
#include <unordered_set>
#include "node.h"
#include "tensor.h"

using std::string;
using std::list;

/* 库所最大token */
const int kPlaceOfMaxTokens = 2;

/* 原模型数据需省略的库所 */
std::unordered_set<int> ignore_m = {
	26, 27, 28
};

std::unordered_set<int> ignore_v = {
	0, 4, 13,
	19, 20, 21, 22, 23, 24, 25,
	26, 27, 28
};

/* Dijkstra 搜索策略 */
class Dijkstra {
public:
	bool operator()(const ptrNode n1, const ptrNode n2) {
		return n1->g_ > n2->g_;
	}
};

/* A* 搜索策略 */
class AStar {
public:
	bool operator()(const ptrNode n1, const ptrNode n2) {
		return n1->g_ + n1->h_pre_ > n2->g_ + n2->h_pre_;
	}
};

class PetriNet {
public:
	/* 库所个数 */
	int num_place_;
	/* 变迁个数 */
	int num_transition_;
	/* 节点池 */
	NodePool pool_;
	/* 根节点 */
	ptrNode root_;
	/* 目标节点 */
	ptrNode goal_node_;
	/* 前置矩阵转置 */
	vector<vector<int>> Tpre_;
	/* 后置矩阵转置 */
	vector<vector<int>> Tpost_;
	/* 关联矩阵转置 */
	vector<vector<int>> C;
	/* 初始标识 */
	vector<int> m0_;
	/* 延时 */
	vector<int> delay_;
	/* 目标标识 */
	vector<int> goal_;
	/* 存储目标节点 */
	vector<ptrNode> goal_nodes_;
	/* 存储死锁节点 */
	std::unordered_map<unsigned, ptrNode> deadlock_nodes_;
	/* 存储叶子节点 */
	std::unordered_map<unsigned, ptrNode> leaf_nodes_;
	/* 存储新节点 open表 */
	std::priority_queue<ptrNode, vector<ptrNode>, Dijkstra> open_list_;
	/* 存储扩展过的节点 close 表 */
	std::map<string, list<ptrNode>> entire_list_;
	/* 四条路径的动作库所 */
	vector<vector<int>> paths_;
	/* 神经网络启发式 */
	std::shared_ptr<Tensor> nn_;

	/* 初始化参数 */
	PetriNet(vector<int>& m, vector<int>& d, vector<int>& goal,
			 vector<vector<int>>& p, vector<vector<int>>& q)
		: pool_(10000000), m0_(m), delay_(d), goal_(goal), nn_(std::make_shared<Tensor>()) {
		if (p.size() == 0 || m0_.size() == 0 || delay_.size() == 0 || goal_.size() == 0 || q.size() == 0) {
			printf("input files error!\n");
			exit(-1);
		}

		num_place_ = m.size();
		num_transition_ = p[0].size();

		Tpre_.resize(num_transition_, vector<int>(num_place_, 0));
		Tpost_.resize(num_transition_, vector<int>(num_place_, 0));
		C.resize(num_transition_, vector<int>(num_place_, 0));

		for (int i = 0; i < num_place_; ++i) {
			int num_of_place = 0;
			for (int j = 0; j < num_transition_; ++j) {
				Tpre_[j][i] = p[i][j];
				Tpost_[j][i] = q[i][j];
				C[j][i] = q[i][j] - p[i][j];
				if (p[i][j] != 0) {
					++num_of_place;
				}
			}
		}

		root_ = pool_.getNode();
		for (int i = 0; i < num_place_; ++i) {
			if (m0_[i] != 0)
				root_->state_.emplace_back(Place(m0_[i], i, vector<int16_t>(kPlaceOfMaxTokens, 0)));
		}

		goal_node_ = pool_.getNode();
		for (int i = 0; i < num_place_; ++i) {
			if (goal_[i] != 0)
				goal_node_->state_.emplace_back(Place(goal_[i], i, vector<int16_t>(kPlaceOfMaxTokens, 0)));
		}

		root_->is_open_ = true;
		open_list_.push(root_);
		std::list<ptrNode> temp = { root_ };
		entire_list_.emplace(root_->toString(), std::move(temp));  // move移动资源，原地址存放的资源不存在

		/* 直线路径包含库所 */
		paths_.assign(4, {});
		/* 路径1中的动作库所 */
		paths_[0] = { 0, 1, 2, 3, 26 };
		/* 路径2中的动作库所 */
		paths_[1] = { 4, 5, 10, 11, 12, 9, 27 };
		/* 路径3中的动作库所 */
		paths_[2] = { 4, 5, 6, 7, 8, 9, 27 };
		/* 路径4中的动作库所 */
		paths_[3] = { 13, 14, 15, 16, 17, 18, 28 };

		return;
	}

	/* 使能变迁判断 */
	vector<int> enableTrans(ptrNode node) {
		vector<int> ans;
		for (int i = 0; i < num_transition_; ++i) {
			if (*node > Tpre_[i])
				ans.push_back(i);
		}
		node->son_ = ans.size();
		return ans;
	}

	/* 目标节点判断 */
	bool isGoalNode(ptrNode curnode) {
		if (*(curnode) > goal_) {
			return 1;
		}
		return 0;
	}

	/* 计算已等待时间v、还需等待最长时间λ */
	int updateVk(ptrNode newnode, ptrNode curnode, int t) {
		int lambda = 0;
		for (int i = 0; i < num_place_; ++i) {
			if (Tpre_[t][i] != 0) {
				// [i]为Node类中的符号重载
				// 意为查看第i个库所是否为空
				// 若为空，返回place->row_ = -1
				// 否则库所i的状态
				if ((*curnode)[i].row_ == -1) {
					printf("something wrong in 110 rows\n");
					exit(-1);
				}
				// 获取输入库所的已等待时间数组
				auto cur_v = (*curnode)[i].v_;
				// v中值最大的指针
				auto v_max = std::max_element(cur_v.begin(), cur_v.end());
				// 还需等待时间
				int diff = delay_[i] - *v_max;
				// 若输入库所激发后还有token 
				if ((*newnode)[i].row_ != -1) {
					// 最大值在v中的索引
					int index = std::distance(cur_v.begin(), v_max);
					// 激发token的已等待时间清零
					(*newnode)[i].v_[index] = 0;
					// 判断v是否从大到小
					bool is_update = std::is_sorted((*newnode)[i].v_.begin(), (*newnode)[i].v_.end(),
						[](int16_t& i1, int16_t& i2) {
							if (i1 == i2) return false;
							return i1 > i2;
						});
					// 若不满足则更新
					if (!is_update) {
						std::sort((*newnode)[i].v_.begin(), (*newnode)[i].v_.end(), std::greater_equal<int16_t>());
					}
				};
				// 还需等待的最大时间
				lambda = std::max(lambda, diff);
			}
		}
		// 更新newnode中的v
		for (int i = 0; i < newnode->state_.size(); ++i) {
			/**
			 * 激发变迁时，Place有两种情况：
			 * 情况一: 静态库所(库所中的token在激发过程中不起作用)
			 *         Tpost_[t][newnode->state_[i].row_] = 0
			 * 情况二：动态库所(库所中的token发生变化)
			 *         Tpost_[t][newnode->state_[i].row_] = 1
			 */
			 // 需更新的token数量
			int num = newnode->state_[i].tokens_ - Tpost_[t][newnode->state_[i].row_];
			for (int j = 0; j < std::min(num, kPlaceOfMaxTokens); ++j) {
				auto& v = newnode->state_[i].v_;
				v[j] = std::min(delay_[newnode->state_[i].row_], v[j] + lambda);
			}
		}
		return lambda;
	}

	/* 启发式(1) */
	float heuristicsOne(ptrNode& node, float idle_time = 0, int ER = 11) {
		// ER:资源个数  time:总时间
		float time = 0;

		// 查找库所id所在路径位置
		auto findPosition = [&](int8_t id) {
			for (int i = 0; i < paths_.size(); ++i) {
				for (int j = 0; j < paths_[i].size(); ++j) {
					if (paths_[i][j] == id) {
						return std::make_pair(i, j);
					}
				}
			}
			return std::make_pair(-1, -1);
		};

		// 累加各个路径上的工件到达各自目标库所的时间
		for (auto& p : node->state_) {
			// 查询结果：存在返回(路径i, 库所j) 不存在返回(-1, -1)
			std::pair<int, int> res = findPosition(p.row_);
			if (res.first == -1) {
				continue;
			}
			// 累加当前库所还需等待的时间
			std::for_each(p.v_.begin(), p.v_.begin() + std::min(2, (int)p.tokens_), [&](int16_t& t) noexcept {
				time += std::max(0, delay_[p.row_] - t);
				});

			// 累加后续库所还需等待的时间
			for (int i = res.second + 1; i < paths_[res.first].size(); ++i) {
				time += delay_[paths_[res.first][i]] * p.tokens_;
			}
		}
		return (time + idle_time) / ER;
	}

	/* 转换为数据集格式进行预测 */
	vector<float> toNetData(const ptrNode node) {
		int j = 0;
		vector<float> ans;
		for (int i = 0; i < num_place_; ++i) {
			if (ignore_m.count(i)) {
				continue;
			}
			if (j < node->state_.size()) {
				if (i < node->state_[j].row_) {
					ans.push_back(0);
				}
				else if (i == node->state_[j].row_) {
					ans.push_back(node->state_[j].tokens_);
					++j;
				}
				else {
					--i; ++j;
				}
			}
			else {
				ans.push_back(0);
			}
		}
		j = 0;
		for (int i = 0; i < num_place_; ++i) {
			if (ignore_v.count(i)) {
				continue;
			}
			if (j < node->state_.size()) {
				if (i < node->state_[j].row_) {
					ans.insert(ans.end(), 2, 0);
				}
				else if (i == node->state_[j].row_) {
					auto v = node->state_[j].v_;
					ans.insert(ans.end(), v.begin(), v.end());
					++j;
				}
				else {
					--i; ++j;
				}
			}
			else {
				ans.insert(ans.end(), 2, 0);
			}
		}
		j = 0;
		for (int i = 0; i < num_place_; ++i) {
			if (ignore_v.count(i)) {
				continue;
			}
			if (j < node->state_.size()) {
				if (i < node->state_[j].row_) {
					ans.push_back(0);
				}
				else if (i == node->state_[j].row_) {
					ans.push_back(delay_[i]);
					++j;
				}
				else {
					--i; ++j;
				}
			}
			else {
				ans.push_back(0);
			}
		}
		return ans;
	}

	/* 变迁激发过程 */
	void fire(ptrNode curnode, int t) {
		auto newnode = *curnode + C[t];
		newnode->discarded_ = false;
		int waiting_time = updateVk(newnode, curnode, t);
		newnode->g_ = curnode->g_ + waiting_time;
		newnode->fathers.emplace_back(std::make_tuple(t, curnode->id_, waiting_time, curnode));
		if (isGoalNode(newnode)) {
			newnode->h_ = 0;
			goal_nodes_.emplace_back(newnode);
			curnode->sons_.emplace(newnode->id_, newnode);
			//newnode->h_pre_ = 0;
			//open_list_.push(newnode);
			return;
		}
		//newnode->h_pre_ = heuristicsOne(newnode);
		//newnode->h_pre_ = nn_->predict(toNetData(newnode));
		auto str = newnode->toString();
		auto pair = isNewNode(newnode);   // return <bool,list<ptrNode>>  bool = 1 or 为新节点  bool = 0 旧节点
		if (pair.second == nullptr) {
			list<ptrNode> temp;
			temp.push_back(newnode);
			curnode->sons_.emplace(newnode->id_, newnode);
			entire_list_.emplace(str, std::move(temp));
			open_list_.push(newnode);
			newnode->is_open_ = true;
		}
		else {
			/* 不是新节点，回收 */
			if (!pair.first) {
				for (auto f : newnode->fathers) {
					std::get<3>(f)->sons_.erase(newnode->id_);
					if ((--std::get<3>(f)->son_) == 0)
						leaf_nodes_.emplace(std::get<3>(f)->id_, std::get<3>(f));
				}
				pool_.recycling(newnode);
				return;
			}
			/* 是新节点，放入open_list和entire_list */
			else {
				curnode->sons_.emplace(newnode->id_, newnode);
				pair.second->emplace_back(newnode);
				open_list_.push(newnode);
				newnode->is_open_ = true;
			}
		}
	}

	/* G方法(是否为旧节点) */
	bool isSame(ptrNode& newnode, ptrNode& oldnode) {
		// 判断v是否相等
		for (int i = 0; i < newnode->state_.size(); ++i) {
			if (newnode->state_[i].v_ != oldnode->state_[i].v_)
				return false;
		}
		// g值比较
		if (newnode->g_ < oldnode->g_) {
			return false;
		}
		// 将状态相同节点合并在一起(m = m', v = v')
		for (auto f : newnode->fathers) {
			if (!std::count(oldnode->fathers.begin(), oldnode->fathers.end(), f)) {
				oldnode->fathers.push_back(f);
				std::get<3>(f)->sons_.emplace(oldnode->id_, oldnode);
				++std::get<3>(f)->son_;
			}
		}
		return true;
	}

	/* GV方法(是否为新节点) */
	bool isNew(ptrNode& newnode, ptrNode& oldnode) {
		unsigned int equal = 0;
		// g值比较
		if (newnode->g_ > oldnode->g_) {
			return false;
		}
		// v向量大小比较
		for (int i = 0; i < newnode->state_.size(); ++i) {
			if (newnode->state_[i].v_ == oldnode->state_[i].v_) {
				++equal;
				continue;
			}
			for (int j = 0; j < newnode->state_[i].v_.size(); ++j) {
				if (newnode->state_[i].v_[j] < oldnode->state_[i].v_[j]) {
					return false;
				}
			}
		}

		// 将标识相同节点合并在一起(m = m')
		if (equal == newnode->state_.size()) {
			for (auto f : oldnode->fathers) {
				// 旧节点的父节点连接至新节点
				if (!std::count(newnode->fathers.begin(), newnode->fathers.end(), f)) {
					newnode->fathers.push_back(f);
					std::get<3>(f)->sons_.emplace(newnode->id_, newnode);
					++std::get<3>(f)->son_;
				}
			}
		}

		return true;
	}

	/* 强时间轴判断(是否为旧节点) */
	bool isOld(ptrNode& newnode, ptrNode& oldnode) {
		unsigned int equal = 0;
		/* g值小则不为旧节点 */
		//if (newnode->g_ < oldnode->g_) {
		//	return false;
		//}
		/* g值差 */
		int diff = newnode->g_ - oldnode->g_;

		// 将v和v'转移到同一时间点比较
		for (int i = 0; i < newnode->state_.size(); ++i) {
			for (int j = 0; j < newnode->state_[i].v_.size(); ++j) {
				if (newnode->state_[i].v_[j] > oldnode->state_[i].v_[j] + diff) {
					return false;
				}
			}
			if (newnode->state_[i].v_ == oldnode->state_[i].v_) {
				++equal;
			}
		}
		// 将旧节点的父节点添加到新节点的父节点中
		if (equal == newnode->state_.size()) {
			for (auto f : newnode->fathers) {
				if (!std::count(oldnode->fathers.begin(), oldnode->fathers.end(), f)) {
					oldnode->fathers.push_back(f);
					std::get<3>(f)->sons_.emplace(oldnode->id_, oldnode);
					++std::get<3>(f)->son_;
				}
			}
		}
		return true;
	}

	/* 新旧节点判断 */
	std::pair<bool, list<ptrNode>*> isNewNode(ptrNode newnode) {
		auto str = newnode->toString();
		if (entire_list_.count(str) <= 0) {
			return std::make_pair(1, nullptr);
		}
		/* 找到相同标识的链表 */
		auto it = entire_list_.find(str);
		/* 依次与链表上的节点进行比较 */
		for (auto itor = it->second.begin(); itor != it->second.end();) {
			auto oldnode = *itor;
			/* 判断新拓展出来节点的新旧性 */
			if (isSame(newnode, oldnode)) {
				return std::make_pair(0, &(it->second));
			}
			/* 判断是否删除旧节点 */
			if (isSame(oldnode, newnode)) {
				/* 只回收没有子节点且不在open表中的状态 */
				if (!oldnode->son_) {
					/* 更新son_ */
					for (auto f : oldnode->fathers) {
						auto f_node = std::get<3>(f);
						if ((--f_node->son_) == 0) {
							leaf_nodes_.emplace(f_node->id_, f_node);
						}
					}
					/* 在死锁、叶子节点表中删除oldnode */
					leaf_nodes_.erase(oldnode->id_); deadlock_nodes_.erase(oldnode->id_);
					/* close表中删除并更新迭代器 */
					itor = it->second.erase(itor);
					oldnode->discarded_ = true;
				}
				else ++itor;
			}
			else ++itor;
		}
		return std::make_pair(1, &(it->second));
	}

	/* 正向树 */
	void forwardTree() {
		std::cout << "\nBegin forward tree -> ";
		clock_t start = clock();
		while (!open_list_.empty()) {
			auto curnode = open_list_.top();
			curnode->is_open_ = false;
			open_list_.pop();

			/* 搜索过程添加 */
			//if (isGoalNode(curnode)) {
			//	break;
			//}

			if (curnode->discarded_) {
				continue;
			}

			auto enables = enableTrans(curnode);
			if (enables.empty()) {
				curnode->is_deadlock_ = true;
				deadlock_nodes_.emplace(curnode->id_, curnode);
				continue;
			}
			for (auto t : enables) {
				fire(curnode, t);
			}
		}
		clock_t end = clock();
		auto programTimes = end - start;
		std::cout << "Forward tree finish（" << programTimes << "ms）->";
	}

	/* 反向树 */
	void backTree() {
		std::cout << "Begin back tree -> ";
		clock_t start = clock();
		vector<ptrNode> back_node = goal_nodes_;

		for (auto& node : deadlock_nodes_) {
			back_node.insert(back_node.end(), node.second);
		}
		for (auto& node : leaf_nodes_) {
			back_node.insert(back_node.end(), node.second);
		}

		/* 搜索过程中获取数据集 */
		//while (!open_list_.empty()) {
		//	auto node = open_list_.top();
		//	open_list_.pop();
		//	if (!isGoalNode(node) && !node->discarded_) {
		//		back_node.push_back(node);
		//	}
		//}

		while (!back_node.empty()) {
			auto node = back_node.back();
			back_node.pop_back();
			for (auto f : node->fathers) {
				auto fnode = std::get<3>(f);
				fnode->h_ = std::min(node->h_ + std::get<2>(f), fnode->h_);

				if ((--fnode->son_) == 0) {
					back_node.push_back(fnode);
				}
			}
		}
		clock_t end = clock();
		auto programTimes = end - start;
		std::cout << "Back tree finish（" << programTimes << "ms）\n" << std::endl;
	}

};