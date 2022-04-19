#include "Graph.h"

Graph::Graph(Position pos, int unitSize, int graphSize):
	graphPosX(pos.first),
	graphPosY(pos.second),
	graphSize(graphSize),
	unitSize(unitSize)
{
	this->InitGraphLine();
}

void Graph::InitGraphLine() {
	int length = this->unitSize * this->graphSize;
	int xPos = 0;
	int yPos = 0;
	for (int i = 0; i < this->graphSize; ++i) {
		xPos = this->graphPosX;
		yPos = this->graphPosY - i * this->unitSize;
		//x軸Lineの構築
		this->graphLines.emplace_back(
			Line(xPos,
				yPos,
				xPos + length,
				yPos
			)
		);
		xPos = this->graphPosX + i * this->unitSize;
		yPos = this->graphPosY;
		//y軸Lineの構築
		this->graphLines.emplace_back(
			Line(
				xPos,
				yPos,
				xPos,
				yPos - length		
			)
		);
	}	
}

void Graph::Put(Circle c) {
	Position converted = this->ConvertPos(Position{ c.x, c.y });
	c.x = converted.first;
	c.y = converted.second;
	this->circles.emplace_back(c);
}

void Graph::Put(Line l) {
	
	Position start = this->ConvertPos(Position{ l.begin.x, l.begin.y});
	Position end = this->ConvertPos(Position{ l.end.x, l.end.y });
	l.begin = Vec2(start.first, start.second);
	l.end = Vec2(end.first, end.second);
	this->lines.emplace_back(l);
}

void Graph::Draw(Line l, Color col) {
	Position start = this->ConvertPos(Position{ l.begin.x, l.begin.y });
	Position end = this->ConvertPos(Position{ l.end.x, l.end.y });
	l.begin = Vec2(start.first, start.second);
	l.end = Vec2(end.first, end.second);
	l.draw(4, col);
}

void Graph::DrawArrow(Line l, Color col) {
	Position start = this->ConvertPos(Position{ l.begin.x, l.begin.y });
	Position end = this->ConvertPos(Position{ l.end.x, l.end.y });
	l.begin = Vec2(start.first, start.second);
	l.end = Vec2(end.first, end.second);
	l.drawArrow(4, Vec2(5, 5), col);
}

void Graph::Draw(Circle c, Color col) {
	Position converted = this->ConvertPos(Position{ c.x, c.y });
	c.x = converted.first;
	c.y = converted.second;
	c.r = c.r * this->unitSize;
	c.draw(col);
}

void Graph::Show() {
	for (const auto& l : this->graphLines) {
		l.draw(1, Palette::Black);
	}
	for (const auto& c : this->circles) {
		c.draw(Palette::Red);
	}
	for (const auto& l : this->lines) {
		l.draw(Palette::Yellow);
	}
}


