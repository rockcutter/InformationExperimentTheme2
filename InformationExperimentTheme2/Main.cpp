﻿#include <Siv3D.hpp> // OpenSiv3D v0.6.3
#include <vector>
#include <array>
#include <stdexcept>
#include <cmath>
#include "MyVec.h"
#include "Graph.h"

//基本パラメータ
constexpr int ROBOT_COUNT = 10; // ロボット数
constexpr double ROBOT_RADIUS = 1.5; //ロボットの直径は3cm

//グラフパラメータ
constexpr int CMPIXEL = 15;//1cmを何pixelで描画するか
constexpr int GRAPH_SIZE = 100;

//ロボットの検知範囲
constexpr double DETECTION_RADIUS = 7;

//各相互作用ルールの検知範囲
constexpr double SEPARATION_RADIUS = 6;
constexpr double ALIGNMENT_RADIUS = 7;
constexpr double COHESION_RADIUS = 7;

//各相互作用ルールの重み
constexpr double PREVIOUS_MOVE_VEC_WEIGHT = 0.5;
constexpr double SEPARATION_WEIGHT = 0.1;
constexpr double ALIGNMENT_WEIGHT = 0.3;
constexpr double COHESION_WEIGHT = 0.1;

//エイリアス
using Position = MyVec<double>;
using MovementVec = MyVec<double>;

//デフォルト移動ベクトルの種類
const MovementVec DEFAULT_VEC_UP{0, 1};
const MovementVec DEFAULT_VEC_RIGHT{1 / std::sqrt(2), 1 / std::sqrt(2)};
const MovementVec DEFAULT_VEC_LEFT{-1 / std::sqrt(2), 1 / std::sqrt(2)};

const Position INITIAL_POSITION_DELTA{ 1, 1 }; //初期位置をずらしたいときに使う

//通常のグラフのような座標管理なのでウィンドウにおける座標の管理ではない
const std::array<Position, ROBOT_COUNT> INITIAL_POS{ // ロボットの初期位置
	std::make_pair<double, double>(0, 2) + INITIAL_POSITION_DELTA,
	std::make_pair<double, double>(2, 6) + INITIAL_POSITION_DELTA,
	std::make_pair<double, double>(4, 0) + INITIAL_POSITION_DELTA,
	std::make_pair<double, double>(6, 12) + INITIAL_POSITION_DELTA,
	std::make_pair<double, double>(8, 8) + INITIAL_POSITION_DELTA,
	std::make_pair<double, double>(8, 4) + INITIAL_POSITION_DELTA,
	std::make_pair<double, double>(10, 0) + INITIAL_POSITION_DELTA,
	std::make_pair<double, double>(12, 10) + INITIAL_POSITION_DELTA,
	std::make_pair<double, double>(14, 6) + INITIAL_POSITION_DELTA,
	std::make_pair<double, double>(16, 0) + INITIAL_POSITION_DELTA,
};
const std::array<MovementVec, ROBOT_COUNT> INITIAL_MOVE_VEC{
	DEFAULT_VEC_RIGHT, 
	DEFAULT_VEC_RIGHT, 
	DEFAULT_VEC_UP,
	DEFAULT_VEC_LEFT,
	DEFAULT_VEC_UP,
	DEFAULT_VEC_LEFT,
	DEFAULT_VEC_RIGHT,
	DEFAULT_VEC_RIGHT,
	DEFAULT_VEC_LEFT,
	DEFAULT_VEC_LEFT
};
const std::array<char, ROBOT_COUNT> INITIAL_IDENTIFIER{
	'a',
	'b',
	'c',
	'd',
	'e',
	'f',
	'g',
	'h',
	'i',
	'j'
};

//ロボットクラス
class Robot {
public:
	char identifier;
	Position pos;
	MovementVec move;
	Robot():identifier(0), pos{0, 0}, move{0, 0}
	{}
};

//Initialize
void InitIdentifiler(std::vector<Robot>& robots) {
	if (robots.size() != ROBOT_COUNT) {
		throw std::logic_error("ロボット数がおかしい");
	}
	for (int i = 0; i < ROBOT_COUNT; ++i) {
		robots[i].identifier = INITIAL_IDENTIFIER[i];
	}
}
//Initialize
void InitRobotPos(std::vector<Robot>& robotArray) {
	if (robotArray.size() != ROBOT_COUNT) {
		throw std::logic_error("ロボット数がおかしい");
	}
	for (int i = 0; i < ROBOT_COUNT; ++i) {
		robotArray[i].pos = INITIAL_POS[i];
	}
}
//Initialize
void InitRobotMoveVec(std::vector<Robot>& robotArray) {
	if (robotArray.size() != ROBOT_COUNT) {
		throw std::logic_error("ロボット数がおかしい");
	}
	for (int i = 0; i < ROBOT_COUNT; ++i) {
		robotArray[i].move = INITIAL_MOVE_VEC[i];
	}
}

//ロボット同士の距離計算
double CalcRobotDistance(const Robot& r1, const Robot& r2) {
	return std::sqrt(std::pow(r1.pos.first - r2.pos.first, 2) + std::pow(r1.pos.second - r2.pos.second, 2));
}

//ノルムの計算
double CalcVecNorm(const MovementVec& vec) {
	return std::sqrt(std::pow(vec.first, 2) + std::pow(vec.second, 2));
}

//単位ベクトルに変換
MovementVec MakeUnitVector(const MovementVec& vec) {
	double norm = CalcVecNorm(MovementVec(vec.first, vec.second));
	if (norm == 0) return { 0, 0 };
	return { vec.first / norm, vec.second / norm };
}

//ベクトルを線分に変換
Line VecToLine(const MovementVec& vec) {
	return Line(vec.first, vec.second, std::atan(vec.first / vec.second), 1);
}

//radiusを半径とする円内のRobotオブジェクトをarrに格納
void GetNearbyRobots(std::vector<Robot>& arr, const Robot& robot, const std::vector<Robot>& robots, double radius) {
	for (const auto& r : robots) {
		if (r.pos == robot.pos) continue;
		if (CalcRobotDistance(robot, r) <= radius) {
			arr.push_back(r);
		}
	}
}

//分離のこと あるロボットとその他すべてのロボットを比較してベクトルを計算
MovementVec Separation(const Robot& robot, const std::vector<Robot>& robots) {
	MovementVec vecSum = std::make_pair<double, double>(0, 0);//加算用のベクトル
	std::vector<Robot> nearbyRobots;

	GetNearbyRobots(nearbyRobots, robot, robots, SEPARATION_RADIUS);
	for (auto& r : nearbyRobots) {
		if (r.pos == robot.pos) continue;
		vecSum.first += r.pos.first - robot.pos.first;
		vecSum.second += r.pos.second - robot.pos.second;
	}

	vecSum = MakeUnitVector(vecSum);
	vecSum.first *= -1;
	vecSum.second *= -1;
	return vecSum;
}

//整列のこと
MovementVec Alignment(const Robot& robot, const std::vector<Robot>& robots) {
	MovementVec vecSum = std::make_pair<double, double>(0, 0);
	std::vector<Robot> nearbyRobots;

	GetNearbyRobots(nearbyRobots, robot, robots, ALIGNMENT_RADIUS);
	for (auto& r : nearbyRobots) {
		if (r.pos == robot.pos) continue;
		vecSum.first += r.move.first;
		vecSum.second += r.move.second;
	}

	vecSum = MakeUnitVector(vecSum);
	return vecSum;
}

//団結のこと
MovementVec Cohesion(const Robot& robot, const std::vector<Robot>& robots) {
	MovementVec vecSum = std::make_pair<double, double>(0, 0);
	std::vector<Robot> nearbyRobots;

	GetNearbyRobots(nearbyRobots, robot, robots, COHESION_RADIUS);

	if (!nearbyRobots.size()) {
		return { 0, 0 };
	}

	for (const auto& r : nearbyRobots) {
		vecSum.first += r.pos.first;
		vecSum.second += r.pos.second;
	}

	//重心の位置ベクトル
	vecSum.first /= nearbyRobots.size();
	vecSum.second /= nearbyRobots.size();

	//重心へのベクトルを計算
	vecSum.first -= robot.pos.first;
	vecSum.second -= robot.pos.second;


	vecSum = MakeUnitVector(vecSum);
	return vecSum;
}

//ロボットの移動(ロボットを現在位置+移動ベクトルの位置へ移動)
void MoveRobots(std::vector<Robot>& robots) {
	for (auto& robot : robots) {
		robot.pos.first += robot.move.first;
		robot.pos.second += robot.move.second;
	}
}

void Main(){
	Window::Resize(Size(1700, 1000));

	//ロボット初期化
	std::vector<Robot> robots(10, Robot());
	InitRobotMoveVec(robots);
	InitRobotPos(robots);
	InitIdentifiler(robots);
	Font font(20);

	//グラフ構築
	Graph graph(Position{10, 800}, CMPIXEL, GRAPH_SIZE);

	//初回か?
	bool isFirstTime = true;
	bool autoMode = false;
	bool eyesightVisualization = false;

	Scene::SetBackground(Palette::White);
	while (System::Update())
	{
		//グラフの表示
		graph.Show();

		//ロボットの表示&移動ベクトルの表示
		for (const auto& robot : robots) {
			double robotx = robot.pos.first;
			double roboty = robot.pos.second;
			//ロボット表示
			graph.Draw(Circle(robotx, roboty, ROBOT_RADIUS), ColorF(1, 0, 0, 0.5));
			//移動ベクトル表示
			graph.Draw(Line(robotx, roboty, robotx + robot.move.first, roboty + robot.move.second), Palette::Green);


			//情報の表示
			Position robotMouseOverCirclePos = graph.ConvertPos(robot.pos);
			if (Circle(robotMouseOverCirclePos.first, robotMouseOverCirclePos.second, 20).mouseOver()) {
				font(robot.identifier,U"\n",
					U"Movement X: ", robot.move.first, U"\n",
					U"Movement Y: ", robot.move.second,U"\n",
					U"Position X: ", robot.pos.first, U"\n"
					U"Position Y: ", robot.pos.second
					).draw(Cursor::Pos().x + 30, Cursor::Pos().y - 20, Palette::Darkblue);
			}

			//視野表示判定
			if (eyesightVisualization) {
				graph.Draw(Circle(robotx, roboty, DETECTION_RADIUS), ColorF(0, 0, 1, 0.2));
			}
		}
		if (SimpleGUI::Button(U"reset", Vec2(250, 550))) {
			isFirstTime = true;
			InitRobotMoveVec(robots);
			InitRobotPos(robots);
		}
		if (SimpleGUI::Button(U"eyesight", Vec2(100, 550))) {
			if (eyesightVisualization) eyesightVisualization = false;
			else eyesightVisualization = true;
		}
		if (SimpleGUI::Button(U"auto", Vec2(400, 550))) {
			if (autoMode) autoMode = false;
			else autoMode = true;
		}
		if (!(SimpleGUI::Button(U"next", Vec2(10, 550)) || autoMode)) {
			continue;
		}
		//初めてなら初期移動ベクトルの通りに移動
		if (isFirstTime) {
			MoveRobots(robots);
			isFirstTime = false;
			continue;
		}

		//ここで分離整列団結を行う ここでrobots内のどの要素も変更してはならない(現在でのパラメータで計算を行ってから動かすため)
		//移動結果はmovedRobotsに格納すること
		std::vector<Robot> movedRobots;
		for (const auto& robot : robots) {
			MovementVec moveVec = robot.move;
			MovementVec tempVec = std::make_pair<double, double>(0, 0);
			//前回の移動ベクトルに重みを掛けたものを保持しておく
			moveVec.first *= PREVIOUS_MOVE_VEC_WEIGHT;
			moveVec.second *= PREVIOUS_MOVE_VEC_WEIGHT;
			//分離操作
			tempVec= Separation(robot, robots);
			moveVec.first += SEPARATION_WEIGHT * tempVec.first;
			moveVec.second += SEPARATION_WEIGHT * tempVec.second;
			//整列操作
			tempVec = Alignment(robot, robots);
			moveVec.first += ALIGNMENT_WEIGHT * tempVec.first;
			moveVec.second += ALIGNMENT_WEIGHT * tempVec.second;
			//団結操作
			tempVec = Cohesion(robot, robots);
			moveVec.first += COHESION_WEIGHT * tempVec.first;
			moveVec.second += COHESION_WEIGHT * tempVec.second;



			if (CalcVecNorm(moveVec) > 1) {
				throw std::logic_error("ベクトルの大きさが1よりデカいぞ(1cm以上進む気だぞ)");
			}

			Robot tempRobot;
			tempRobot.identifier = robot.identifier;
			tempRobot.pos = robot.pos;
			tempRobot.move = moveVec;
			movedRobots.push_back(tempRobot);

			double robotx = robot.pos.first;
			double roboty = robot.pos.second;
			graph.Draw(Line(robotx, roboty, robotx + robot.move.first, roboty + robot.move.second), Palette::Red);
		}

		robots = movedRobots;
		MoveRobots(robots);
	}
}
