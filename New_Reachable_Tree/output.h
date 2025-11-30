#pragma once
/**
 * 生成数据txt、可视化文件gv
 */

#include <algorithm>
#include <iostream>
#include <fstream>
#include <unordered_set>
#include <string>
#include <queue>
#include <iomanip>
#include <functional>
#include <mutex>
#include <cmath>
#include "petri.h"
#include "input.h"

static std::mutex mtx;

/* 原模型数据需省略的库所 */
//std::unordered_set<int> ignore_m = {
//	26, 27, 28
//};
//
//std::unordered_set<int> ignore_v = {
//	0, 4, 13,
//	19, 20, 21, 22, 23, 24, 25,
//	26, 27, 28
//};

/* 修改后模型数据需省略的库所 */
//static std::unordered_set<int> ignore_m = { 
//	26, 27, 28, 29, 30, 31, 32, 33,
//	34, 35, 36, 37, 38, 39, 40, 41,
//	42, 43, 44, 45, 46 
//};
//static std::unordered_set<int> ignore_v = {
//	0, 4, 13, 19, 20, 21, 22, 23,
//	24, 25, 26, 27, 28, 29, 30, 31,
//	32, 33, 34, 35, 36, 37, 38, 39,
//	40, 41, 42, 43, 44, 45, 46
//};

/* 数据量限制倍数 */
static float max_accept_multiple = 1;
/* tree_nodes_num: 可达图节点数量  total_nodes_num: 保留数据量 */
static unsigned long tree_nodes_num, total_nodes_num;

/* 生成全局可达图 */
void globalGraphCreate(PetriNet& tree) {
	/* 加锁 */
	mtx.lock();
	/* 数据保留阈值 */
	auto threshold_value = max_accept_multiple * tree.root_->h_;
	/* 图节点输出流 */
	std::ofstream file(kOutputGvPath);

	std::unordered_set<unsigned int> uset;

	//std::unordered_set<int> goals;
	//std::for_each(tree.goal_nodes_.begin(), tree.goal_nodes_.end(), [&](ptrNode& node) {
	//	if (goals.find(node->g_) == goals.end()) {
	//		goals.emplace(node->g_);
	//	}
	//	});

	file << "digraph {" << '\n';
	/* 设置节点 */
	for (auto& node : tree.goal_nodes_) {
		if (node->g_ + node->h_ <= tree.root_->h_ * max_accept_multiple) {
			file << "\""
				<< "m" << node->id_
				<< "_g" << node->g_
				<< "_h" << node->h_
				<< "\""
				<< " [color=red style=filled]"
				<< "\n";
		}
	}
	for (auto& nodes : tree.entire_list_) {
		for (auto& node : nodes.second) {
			if (node->g_ + node->h_ <= tree.root_->h_ * max_accept_multiple) {
				if (tree.deadlock_nodes_.count(node->id_) > 0) {
					file << "\""
						<< "m" << node->id_
						<< "_g" << node->g_
						<< "_h" << node->h_
						<< "\""
						<< " [color=blue]"
						<< "\n";
				}
				else if (tree.leaf_nodes_.count(node->id_) > 0) {
					file << "\""
						<< "m" << node->id_
						<< "_g" << node->g_
						<< "_h" << node->h_
						<< "\""
						<< " [color=green]"
						<< "\n";
				}
				else {
					file << "\""
						<< "m" << node->id_
						<< "_g" << node->g_
						<< "_h" << node->h_
						<< "\""
						<< '\n';
				}
				for (auto& n1 : node->sons_) {
					if (n1.second->g_ + n1.second->h_ <= tree.root_->h_ * max_accept_multiple)
						break;
					if (tree.leaf_nodes_.count(n1.second->id_) > 0) {
						file << "\""
							<< "m" << n1.second->id_
							<< "_g" << n1.second->g_
							<< "_h" << (n1.second->h_ < 9999 ?
								n1.second->h_ : tree.root_->h_ * 1.05 - n1.second->g_)
							<< "\""
							<< " [color=green]"
							<< "\n";
					}
					else {
						file << "\""
							<< "m" << n1.second->id_
							<< "_g" << n1.second->g_
							<< "_h" << (n1.second->h_ < 9999 ?
								n1.second->h_ : tree.root_->h_ * 1.05 - n1.second->g_)
							<< "\""
							<< '\n';
					}
					file << "\""
						<< "m" << node->id_
						<< "_g" << node->g_
						<< "_h" << node->h_
						<< "\""
						<< " -> "
						<< "\""
						<< "m" << n1.second->id_
						<< "_g" << n1.second->g_
						<< "_h" << (n1.second->h_ < 9999 ?
							n1.second->h_ : tree.root_->h_ * 1.05 - n1.second->g_)
						<< "\""
						<< " [label=t" << std::get<0>(n1.second->fathers[0]) << "]\n";
				}
			}
		}
	}


	/* 设置节点间的连接弧 */
	/* 思想：从该节点找到其父节点 并进行连接 */
	for (auto& node : tree.goal_nodes_) {
		if (node->g_ + node->h_ <= tree.root_->h_ * max_accept_multiple) {
			for (auto& father : node->fathers) {
				auto f_node = std::get<3>(father);
				file << "\""
					<< "m" << f_node->id_
					<< "_g" << f_node->g_
					<< "_h" << f_node->h_
					<< "\""
					<< " -> "
					<< "\""
					<< "m" << node->id_
					<< "_g" << node->g_
					<< "_h" << node->h_
					<< "\""
					<< " [label=t" << std::get<0>(father) << "]\n";
			}
		}
	}
	for (auto& nodes : tree.entire_list_) {
		for (auto& node : nodes.second) {
			if (node->g_ + node->h_ <= tree.root_->h_ * max_accept_multiple) {
				for (auto& father : node->fathers) {
					auto f_node = std::get<3>(father);
					if (f_node->g_ + f_node->h_ <= tree.root_->h_ * max_accept_multiple) {
						file << "\""
							<< "m" << f_node->id_
							<< "_g" << f_node->g_
							<< "_h" << f_node->h_
							<< "\""
							<< " -> "
							<< "\""
							<< "m" << node->id_
							<< "_g" << node->g_
							<< "_h" << node->h_
							<< "\""
							<< " [label=t" << std::get<0>(father) << "]\n";
					}
				}
			}
		}
	}
	file << "}";
	file.close();
	/* 解锁 */
	mtx.unlock();
}

/* 输出到达目标路径 */
void graphCreate(PetriNet& tree) {
	std::queue<ptrNode> back_node;
	std::unordered_set<unsigned short> already;
	short num_of_goals = 0;
	std::ofstream ofs(kOutputGvPath, std::ios::out);
	ofs << "digraph {" << '\n';
	for (auto& node : tree.goal_nodes_) {
		back_node.push(node);
	}
	while (!back_node.empty()) {
		auto node = back_node.front();
		back_node.pop();

		if (num_of_goals < tree.goal_nodes_.size()) {
			num_of_goals++;
			ofs << "m" << node->id_
				<< "_g" << node->g_
				<< "_h" << node->h_
				<< " [color=red, style=filled]"
				<< "\n";
		}
		else {
			ofs << "m" << node->id_
				<< "_g" << node->g_
				<< "_h" << node->h_
				<< "\n";
		}

		if (node->fathers.empty()) {
			continue;
		}

		auto f_node = std::get<3>(node->fathers[0]);
		ofs << "m" << f_node->id_
			<< "_g" << f_node->g_
			<< "_h" << f_node->h_
			<< " -> "
			<< "m" << node->id_
			<< "_g" << node->g_
			<< "_h" << node->h_
			<< " [label=t" << std::get<0>(node->fathers[0])
			<< "]\n";
		if (already.count(f_node->id_) == 0) {
			back_node.push(f_node);
			already.emplace(f_node->id_);
		}
	}
	ofs << "}";
	ofs.close();
}

template <typename T>
/* 转换为数据集格式 */
vector<T> toVector(const ptrNode node, const int num_place, const vector<int>& delay) {   // 节点指针  PetriNet库所总数
	auto places = node->state_;
	int j = 0;   // 对已赋值的库所进行计数
	std::vector<T> ans;
	for (int i = 0; i < num_place; ++i) {   //  遍历所有库所，进行赋值
		if (ignore_m.count(i) >= 1)    // 若查到找ignore_m中存储的无用库所，则跳过不赋值
			continue;
		if (j < places.size()) {       // 计数值未到达需赋值的库所数
			if (i < places[j].row_) {  // 将state_中未记录的库所赋予0
				ans.push_back(0);
			}
			else if (i == places[j].row_) {      // 到达state_中包含的place，对token数进行赋值
				ans.push_back(places[j].tokens_);
				++j;   // 计数值+1
			}
			else {     // 由于上面有continue，会出现无赋值且i自动加1，导致i位置空出；ignore_m中包含库所存在于state_中；
				--i;   // 所以利用--i跳到空位置；++j补上含有的place计数值，否则j无法==含有token的库所数
				++j;
			}
		}
		else {         // 若计数值=含有token的库所数，则剩余的库所全置0
			ans.push_back(0);
		}
	}
	j = 0;

	/* 与上述同理 */
	for (int i = 0; i < num_place; ++i) {
		if (ignore_v.count(i) >= 1)
			continue;
		if (j < places.size()) {
			if (i < places[j].row_) {
				ans.insert(ans.end(), 2, 0);
			}
			else if (i == places[j].row_) {
				ans.insert(ans.end(), places[j].v_.begin(), places[j].v_.end());
				++j;
			}
			else {
				--i;
				++j;
			}
		}
		else {
			ans.insert(ans.end(), 2, 0);
		}
	}
	j = 0;

	/* 加入延时信息 */
	for (int i = 0; i < num_place; ++i) {
		if (ignore_v.count(i) >= 1)
			continue;
		if (j < places.size()) {
			if (i < places[j].row_) {
				ans.push_back(0);
			}
			else if (i == places[j].row_) {
				ans.push_back(delay[i]);
				++j;
			}
			else {
				--i;
				++j;
			}
		}
		else {
			ans.push_back(0);
		}
	}

	return ans;
}

/* 生成目标路径及其附近的数据 */
void dataCreateTxt(PetriNet& tree, const int num_place) {
	/* 加锁 */
	mtx.lock();
	/* 数据集输出流 */
	std::ofstream file(kOutputTxtPath);
	/* 保留两位小数格式 */
	file << std::setiosflags(std::ios::fixed) << std::setprecision(2);

	// 存放已输出的节点标号
	std::unordered_set<unsigned int> uset;
	// 存放最优路径上的节点
	std::queue<ptrNode> datas;
	datas.push(tree.root_);

	auto Generate = [&](ptrNode n) {
		static bool flag = false;
		vector<float> vec = toVector<float>(n, num_place, tree.delay_);

		if (!flag) {
			flag = true;
		}
		else {
			file << '\n';
		}

		for (auto& v : vec) {
			file << v << '\t';
		}

		file << (n->h_ < 9999 ? n->h_ : tree.root_->h_ * 1.1 - n->g_);

		++total_nodes_num;
	};

	// 初始化
	Generate(tree.root_);

	while (!datas.empty()) {
		// 取出节点
		auto node = datas.front();
		datas.pop();

		for (auto son : node->sons_) {
			if (uset.count(son.second->id_)) {
				continue;
			}

			// 输出一条数据
			Generate(son.second);
			uset.insert(son.second->id_);

			if (son.second->g_ + son.second->h_ == tree.root_->h_) {
				datas.push(son.second);
			}
			// 拓展两步
			else {
				for (auto _son : son.second->sons_) {
					if (uset.count(_son.second->id_) ||
						_son.second->g_ + _son.second->h_ == tree.root_->h_) {
						continue;
					}
					// 输出一条数据
					Generate(_son.second);
					uset.insert(_son.second->id_);
				}
			}
		}
	}

	file.close();
	/* 解锁 */
	mtx.unlock();
}

/* 生成全局数据 */
void dataCreateAllTxt(PetriNet& tree, const int num_place) {
	/* 加锁 */
	mtx.lock();
	/* 数据集输出流 */
	std::ofstream file(kOutputTxtPath);
	/* 保留两位小数格式 */
	file << std::setiosflags(std::ios::fixed) << std::setprecision(2);

	auto OutputData = [&](ptrNode node) {
		/* 转换为数据格式 */
		auto vec = toVector<float>(node, num_place, tree.delay_);
		for (auto v : vec) {
			file << v << '\t';
		}
		file << (node->h_ < 9999 ? node->h_ : tree.root_->h_ * 1.10 - node->g_);
		/* 计数 */
		++total_nodes_num;
	};

	bool begin = true;
	/* 目标节点 */
	for (auto& node : tree.goal_nodes_) {
		/* 起始不需要换行 */
		if (!begin) {
			file << '\n';
		}
		else {
			begin = false;
		}
		/* 输出数据 */
		OutputData(node);
	}

	/* 其余节点 */
	for (auto& nodes : tree.entire_list_) {
		for (auto& node : nodes.second) {
			if (node->h_ > 9999) {
				/* 查看其父节点是否在最优路径上 */
				bool flag = std::any_of(
					node->fathers.begin(), node->fathers.end(),
					[&](decltype(node->fathers[0])& t) {
						ptrNode fnode = std::get<3>(t);
						return fnode->h_ + fnode->g_ == tree.root_->h_;
					});

				if (!flag) continue;
			}
			/* 换行 */
			file << '\n';
			/* 输出数据 */
			OutputData(node);
		}
	}

	file.close();
	/* 解锁 */
	mtx.unlock();
}

/* 信息输出 */
void infoCreate(PetriNet& tree) {
	/* 加锁 */
	mtx.lock();
	/* 信息文件输出流 */
	/*std::ofstream file(kInfoPath, std::ios::out);

	tree_nodes_num += tree.goal_nodes_.size();
	for (auto nodes : tree.entire_list_) {
		tree_nodes_num += nodes.second.size();
	}

	if (!file.is_open()) {
		std::cerr << "File open error!" << std::endl;
		exit(-1);
	}
	else {
		file << "Total nodes = " << tree_nodes_num << '\n';
		file << "Target nodes  = " << tree.goal_nodes_.size() << '\n';
		file << "Dataset = " << total_nodes_num << '\n';
		file << "The minimum time = " << tree.root_->h_ << "s\n";
		file << "Node size = " << sizeof(Node);
	}*/

	std::cout.setf(std::ios::fixed);
	std::cout.precision(2);

	for (auto nodes : tree.entire_list_) {
		for (auto node : nodes.second) {
			if (node->son_) {
				std::cout << node->id_ << '\t' << node->son_ << "\tBackTree exception\n";
			}
			std::cout << "节点id: " << node->id_ << '\t';
			std::cout << "实际值: " << node->h_ << '\t';
			std::cout << "传统启发式h1: " << tree.heuristicsOne(node) << '\t';
			std::cout << "绝对误差: " << std::abs(tree.heuristicsOne(node) - node->h_) << "    ";
			std::cout << "神经网络启发式hDNN: " << tree.nn_->predict(tree.toNetData(node)) << '\t';
			std::cout << "绝对误差: " << std::abs(tree.nn_->predict(tree.toNetData(node)) - node->h_) << '\n';
		}
	}

	std::cout << "deadlock nodes = " << tree.deadlock_nodes_.size() << "   ";
	std::cout << "leaf nodes = " << tree.leaf_nodes_.size() << "   ";
	std::cout << "Goal nodes = " << tree.goal_nodes_.size() << '\n' << std::endl;
	for (auto &goal_ : tree.goal_nodes_) {
		std::cout << "Mark：" << goal_->toString() << "  g = " << goal_->g_ << '\n';
	}

	//file.close();
	/* 解锁 */
	mtx.unlock();
}