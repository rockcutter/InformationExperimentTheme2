# include <Siv3D.hpp> // OpenSiv3D v0.6.3
#include <array>
#include <stdexcept>

constexpr int ROBOT_COUNT = 10; // ロボット数
constexpr int CMPIXEL = 10;//1cmを何pixelで描画するか
constexpr double ROBOT_RADIUS = CMPIXEL * 3; //ロボットの直径は3cm

using Position = std::pair<double, double>;
using MovementVec = std::pair<double, double>;

const Position INITIAL_POSITION_DELTA{ 0, 0 }; //初期位置をずらしたいときに使う
const std::array<Position, ROBOT_COUNT> INITIAL_POS{ // ロボットの初期位置
	std::make_pair<int, int>(0, 1),
	std::make_pair<int, int>(2, 0),
	std::make_pair<int, int>(1, 3),
	std::make_pair<int, int>(2, 0),
	std::make_pair<int, int>(2, 0),
	std::make_pair<int, int>(2, 0),
	std::make_pair<int, int>(2, 0),
	std::make_pair<int, int>(2, 0),
	std::make_pair<int, int>(2, 0),
	std::make_pair<int, int>(2, 0),
};

class Robot {
public:
	Position pos;
	MovementVec move;
	Robot(): pos{0, 0}, move{0, 0}
	{}
};

//初期座標をセットしている
void InitRobotPos(std::vector<Robot>& robotArray) {
	if (robotArray.size() != ROBOT_COUNT) {
		throw std::logic_error("ロボットサイズがおかしい");
	}
	for (int i = 0; i < ROBOT_COUNT; ++i) {
		robotArray[i].pos = INITIAL_POS[i];
	}
}

void Main(){
	std::vector<Robot> robots(10, Robot());

	while (System::Update())
	{
	}
}

//
// = アドバイス =
// Debug ビルドではプログラムの最適化がオフになります。
// 実行速度が遅いと感じた場合は Release ビルドを試しましょう。
// アプリをリリースするときにも、Release ビルドにするのを忘れないように！
//
// 思ったように動作しない場合は「デバッグの開始」でプログラムを実行すると、
// 出力ウィンドウに詳細なログが表示されるので、エラーの原因を見つけやすくなります。
//
// = お役立ちリンク | Quick Links =
//
// Siv3D リファレンス
// https://zenn.dev/reputeless/books/siv3d-documentation
//
// Siv3D Reference
// https://zenn.dev/reputeless/books/siv3d-documentation-en
//
// Siv3D コミュニティへの参加
// Slack や Twitter, BBS で気軽に質問や情報交換ができます。
// https://zenn.dev/reputeless/books/siv3d-documentation/viewer/community
//
// Siv3D User Community
// https://zenn.dev/reputeless/books/siv3d-documentation-en/viewer/community
//
// 新機能の提案やバグの報告 | Feedback
// https://github.com/Siv3D/OpenSiv3D/issues
//
