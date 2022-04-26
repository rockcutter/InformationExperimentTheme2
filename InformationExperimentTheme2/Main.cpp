#include <Siv3D.hpp> // OpenSiv3D v0.6.3
#include <vector>
#include <array>
#include <stdexcept>
#include <cmath>
#include "MyVec.h"
#include "Graph.h"

//エイリアス
using Position = MyVec<double>;
using MovementVec = MyVec<double>;

//基本パラメータ
constexpr int ROBOT_COUNT = 10; // ロボット数
constexpr double ROBOT_RADIUS = 1.5; //ロボットの直径は3cm

//グラフパラメータ
constexpr int CMPIXEL = 15;//1cmを何pixelで描画するか
constexpr int GRAPH_SIZE = 100;
constexpr int GRAPH_X = 10;
constexpr int GRAPH_Y = 500;

//ロボットの検知範囲
constexpr double DETECTION_RADIUS = 7;

//各相互作用ルールの検知範囲
constexpr double SEPARATION_RADIUS = 6;
constexpr double ALIGNMENT_RADIUS = 7;
constexpr double COHESION_RADIUS = 7;

//各相互作用ルールの重み
double PREVIOUS_MOVE_VEC_WEIGHT = 0.5;
double SEPARATION_WEIGHT = 0.1;
double ALIGNMENT_WEIGHT = 0.3;
double COHESION_WEIGHT = 0.1;


//デフォルト移動ベクトルの種類
const MovementVec DEFAULT_VEC_UP{0, 1};
const MovementVec DEFAULT_VEC_RIGHT{1 / std::sqrt(2), 1 / std::sqrt(2)};
const MovementVec DEFAULT_VEC_LEFT{-1 / std::sqrt(2), 1 / std::sqrt(2)};

const Position INITIAL_POSITION_DELTA{ 0, 0 }; //初期位置をずらしたいときに使う

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
	Robot(char identifier, Position pos, MovementVec move) 
		: identifier(identifier), pos(pos), move(move)
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
	Position distance = r1.pos - r2.pos;
	return std::sqrt(std::pow(distance.first, 2) + std::pow(distance.second, 2));
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

//radiusを半径とする円内のRobotオブジェクトをarrに格納(自分含む)
void GetNearbyRobots(std::vector<Robot>& arr, const Robot& robot, const std::vector<Robot>& robots, double radius) {
	for (const auto& r : robots) {
		if (CalcRobotDistance(robot, r) <= radius) {
			arr.push_back(r);
		}
	}
}

//分離のこと あるロボットとその他すべてのロボットを比較してベクトルを計算
MovementVec Separation(const Robot& robot, const std::vector<Robot>& robots) {
	MovementVec vecSum = std::make_pair<double, double>(0, 0);//加算用のベクトル
	std::vector<Robot> nearbyRobots;

	//一定範囲内の近くのロボットを探索(自分含む)
	GetNearbyRobots(nearbyRobots, robot, robots, SEPARATION_RADIUS);

	//範囲内のロボット(自分以外)に対する自分からのベクトルを求めてvecSumに加算しておく
	for (auto& r : nearbyRobots) {
		if (r.pos == robot.pos) continue;
		vecSum = vecSum + r.pos - robot.pos;
	}
	
	vecSum = MakeUnitVector(vecSum);
	vecSum = vecSum * MovementVec{ -1, -1 }; //自分から近くのロボットへのベクトルを加算した結果を反転している
	return vecSum;
}

//整列のこと
MovementVec Alignment(const Robot& robot, const std::vector<Robot>& robots) {
	MovementVec vecSum = std::make_pair<double, double>(0, 0);
	std::vector<Robot> nearbyRobots;

	//一定範囲内のロボットを探索(自分含む)
	GetNearbyRobots(nearbyRobots, robot, robots, ALIGNMENT_RADIUS);
	//自分を除く一定範囲内のロボットの移動ベクトルをすべて加算
	for (auto& r : nearbyRobots) {
		if (r.pos == robot.pos) continue;
		vecSum = vecSum + r.move;
	}

	vecSum = MakeUnitVector(vecSum);
	return vecSum;
}

//団結のこと
MovementVec Cohesion(const Robot& robot, const std::vector<Robot>& robots) {
	MovementVec vecSum = std::make_pair<double, double>(0, 0);
	std::vector<Robot> nearbyRobots;

	//一定範囲内の以下略
	GetNearbyRobots(nearbyRobots, robot, robots, COHESION_RADIUS);

	//近くにロボットが無ければ0を返す
	if (!nearbyRobots.size()) {
		return {0, 0};
	}

	//近くのロボット(自分含む)の位置座標をvecSumに加算しておく
	for (const auto& r : nearbyRobots) {
		vecSum = vecSum + r.pos;
	}

	//重心の位置ベクトルをもとめる
	vecSum.first /= nearbyRobots.size();
	vecSum.second /= nearbyRobots.size();

	//重心へのベクトルを計算
	vecSum = vecSum - robot.pos;

	vecSum = MakeUnitVector(vecSum);
	return vecSum;
}

//ロボットの移動(ロボットを現在位置+移動ベクトルの位置へ移動)
void MoveRobots(std::vector<Robot>& robots) {
	for (auto& robot : robots) {
		robot.pos = robot.pos + robot.move;
	}
}

void Main(){
	Window::Resize(Size(1700, 1000));
	Scene::SetBackground(Palette::White);

	Font font(20);

	//ロボット初期化
	std::vector<Robot> robots(10, Robot());
	InitRobotMoveVec(robots);
	InitRobotPos(robots);
	InitIdentifiler(robots);

	//グラフ構築
	Graph graph(Position{GRAPH_X, GRAPH_Y}, CMPIXEL, GRAPH_SIZE);

	//各種フラグ
	bool isFirstTime = true;
	bool autoMode = false;
	bool eyesightVisualization = false;
	bool settingMode = false;

	int loopCount = 0;

	//texteditstate
	constexpr int TES_COUNT = 4;
	std::vector<TextEditState> teses(TES_COUNT);
	const std::vector<String> SETTING_OPTION_NAME{
		U"前回の移動ベクトルの重み: ",
		U"整列の重み",
		U"団結の重み",
		U"分離の重み"
	};

	//メインループ
	while (System::Update())
	{
		//描写等の処理
		//ロボット制御プログラムへ移行する時はここから気にしなくてよい------------------------------
		font(loopCount).draw(600, 550, Palette::Black);
		//setting画面
		if (settingMode) {
			if (SimpleGUI::Button(U"閉じる", Vec2(10, 550))) {
				try {
					PREVIOUS_MOVE_VEC_WEIGHT	= Parse<double>(teses[0].text);
					ALIGNMENT_WEIGHT			= Parse<double>(teses[1].text);
					COHESION_WEIGHT				= Parse<double>(teses[2].text);
					SEPARATION_WEIGHT			= Parse<double>(teses[3].text);
					settingMode = false;
				}
				catch (const ParseError&) {
				}
				continue;
			}
			for (int i = 0; i < TES_COUNT; ++i) {
				SimpleGUI::TextBox(teses[i], Vec2(10, 20 + i * 50), 50);
				font(SETTING_OPTION_NAME[i]).draw(70, 20 + i * 50 + 5, Palette::Black);
			}
			continue;
		}

		//グラフの表示
		graph.Show();

		//ロボットの表示&移動ベクトルの表示
		for (const auto& robot : robots) {

			double robotx = robot.pos.first;
			double roboty = robot.pos.second;
			//ロボット表示
			graph.Draw(Circle(robotx, roboty, ROBOT_RADIUS), ColorF(1, 0, 0, 0.5));
			//移動ベクトル表示
			graph.Draw(Line(robotx, roboty, robotx + robot.move.first, roboty + robot.move.second),Palette::Green);

			//情報の表示
			Position robotMouseOverCirclePos = graph.ConvertPos(robot.pos);
			if (Circle(robotMouseOverCirclePos.first, robotMouseOverCirclePos.second, 20).mouseOver()) {
				Point cursorPos = Cursor::Pos();
				Rect(cursorPos.x + 30, cursorPos.y - 20, 220, 180).draw(Palette::White);
				font(robot.identifier,U"\n",
					U"Movement X: ", robot.move.first, U"\n",
					U"Movement Y: ", robot.move.second,U"\n",
					U"Movement norm: ", CalcVecNorm(robot.move), U"\n",
					U"Position X: ", robot.pos.first, U"\n"
					U"Position Y: ", robot.pos.second
					).draw(cursorPos.x + 30, cursorPos.y - 20, Palette::Darkblue);
			}

			//視野表示判定
			if (eyesightVisualization) {
				graph.Draw(Circle(robotx, roboty, DETECTION_RADIUS), ColorF(0, 0, 1, 0.2));
				graph.Draw(Circle(robotx, roboty, SEPARATION_RADIUS), ColorF(0, 1, 0, 0.2));
			}
		}
		if (SimpleGUI::Button(U"Setting", Vec2(450, 550))) {
			teses[0].text = Format(PREVIOUS_MOVE_VEC_WEIGHT);
			teses[1].text = Format(ALIGNMENT_WEIGHT);
			teses[2].text = Format(COHESION_WEIGHT);
			teses[3].text = Format(SEPARATION_WEIGHT);
			settingMode = true;
		}
		if (SimpleGUI::Button(U"reset", Vec2(230, 550))) {
			isFirstTime = true;
			loopCount = 0;
			InitRobotMoveVec(robots);
			InitRobotPos(robots);
		}
		if (SimpleGUI::Button(U"eyesight", Vec2(100, 550))) {
			if (eyesightVisualization) eyesightVisualization = false;
			else eyesightVisualization = true;
		}
		if (SimpleGUI::Button(U"auto", Vec2(340, 550))) {
			if (autoMode) autoMode = false;
			else autoMode = true;
		}
		if (!(SimpleGUI::Button(U"next", Vec2(10, 550)) || autoMode)) {
			continue;
		}
		++loopCount;
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		//ロボット制御プログラムへ移行する時はここまで気にしなくてよい---------------------------------------
		//描写等の処理ここまで


		
		//①初めてなら初期移動ベクトルの通りに移動
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

			//②前回の移動ベクトルに重みを掛けたものを保持しておく
			moveVec.first *= PREVIOUS_MOVE_VEC_WEIGHT;
			moveVec.second *= PREVIOUS_MOVE_VEC_WEIGHT;
			//③分離操作
			tempVec= Separation(robot, robots);
			moveVec.first += SEPARATION_WEIGHT * tempVec.first;
			moveVec.second += SEPARATION_WEIGHT * tempVec.second;
			//④整列操作
			tempVec = Alignment(robot, robots);
			moveVec.first += ALIGNMENT_WEIGHT * tempVec.first;
			moveVec.second += ALIGNMENT_WEIGHT * tempVec.second;
			//⑤団結操作
			tempVec = Cohesion(robot, robots);
			moveVec.first += COHESION_WEIGHT * tempVec.first;
			moveVec.second += COHESION_WEIGHT * tempVec.second;


			if (CalcVecNorm(moveVec) > 1) {
				throw std::logic_error("ベクトルの大きさが1よりデカいぞ(1cm以上進む気だぞ)");
			}
			
			movedRobots.emplace_back(Robot(robot.identifier, robot.pos, moveVec));
		}

		//robotsを更新
		robots = movedRobots;
		MoveRobots(robots);
	}
}
