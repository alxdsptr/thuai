#include <vector>
#include <thread>
#include <array>
#include <queue>
#include <map>
#include <cstring>
#include "AI.h"
#include "constants.h"
#define maxLen 55
// 注意不要使用conio.h，Windows.h等非标准库
// 为假则play()期间确保游戏状态不更新，为真则只保证游戏状态在调用相关方法时不更新，大致一帧更新一次
extern const bool asynchronous = false;
constexpr double PI = 3.1415926;

// 选手需要依次将player1到player4的船类型在这里定义
extern const std::array<THUAI7::ShipType, 4> ShipTypeDict = {
    THUAI7::ShipType::CivilianShip,
    THUAI7::ShipType::MilitaryShip,
    THUAI7::ShipType::MilitaryShip,
    THUAI7::ShipType::FlagShip,
};

// 可以在AI.cpp内部声明变量与函数
struct coordinate
{
    int x, y;
};
struct node
{
    int x, y;
    coordinate from;
    double cost, heuristic;
    bool operator<(const node& a) const
    {
		return cost + heuristic > a.cost + a.heuristic;
	}
};
bool moved = false;
bool visited[maxLen][maxLen];
coordinate from[maxLen][maxLen];

THUAI7::PlaceType map[maxLen][maxLen];

int manhatten_distance(int x1, int y1, int x2, int y2)
    { return abs(x1 - x2) + abs(y1 - y2); };
int manhatten_distance(int x1, int y1, coordinate c)
    { return abs(x1 - c.x) + abs(y1 - c.y); };
std::deque<coordinate> search_road(int x1, int y1, int x2, int y2)
{

    std::priority_queue<node> pq;
    pq.push({x1, y1, coordinate{-1, -1}, 0, (double)manhatten_distance(x1, y1, x2, y2)});
    memset(visited, false, sizeof(visited));
    while (!pq.empty())
    {
		node now = pq.top();
		pq.pop();
		if (visited[now.x][now.y])
			continue;
		visited[now.x][now.y] = true;
        from[now.x][now.y] = now.from;
        if (now.x == x2 && now.y == y2)
        {
            std::deque<coordinate> path;
            path.push_back({now.x, now.y});
            coordinate temp = now.from;
            while (temp.x != -1)
			{
                path.push_front({temp.x, temp.y});
                temp = from[temp.x][temp.y];
            }
            path.pop_front();
            return path;
        }
        auto is_empty = [](THUAI7::PlaceType t)
        {
            return t == THUAI7::PlaceType::Space or t == THUAI7::PlaceType::Shadow or t == THUAI7::PlaceType::Wormhole;
        };
        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {
                if (i == 0 and j == 0)
                    continue;
                if (i == -1 and j == -1 and (!is_empty(map[now.x - 1][now.y]) or !is_empty(map[now.x][now.y - 1]))) continue;
                if (i == 1 and j == -1 and (!is_empty(map[now.x + 1][now.y]) or !is_empty(map[now.x][now.y - 1]))) continue;
                if (i == -1 and j == 1 and (!is_empty(map[now.x - 1][now.y]) or !is_empty(map[now.x][now.y + 1]))) continue;
                if (i == 1 and j == 1 and (!is_empty(map[now.x + 1][now.y]) or !is_empty(map[now.x][now.y + 1]))) continue;
			    int nx = now.x + i, ny = now.y + j;
                if (nx < 0 or nx >= 50 or ny < 0 or ny >= 50 or visited[nx][ny])
                    continue;
                if (nx == x2 and ny == y2)
                {
                    double cost = sqrt(i * i + j * j);
                    pq.push({nx, ny, coordinate{now.x, now.y}, now.cost + cost, 0});
                }
                THUAI7::PlaceType t = map[nx][ny];
                if (t != THUAI7::PlaceType::Space and t != THUAI7::PlaceType::Shadow and t != THUAI7::PlaceType::Wormhole)
					continue;
                double cost = sqrt(i * i + j * j);
                int h = manhatten_distance(nx, ny, x2, y2);
                pq.push({nx, ny, coordinate{now.x, now.y}, now.cost + cost, (double)h});
            }
        }
    }
}
std::deque<coordinate> search_road(int x1, int y1, THUAI7::PlaceType type)
{
    std::vector<coordinate> des;
    int min_dis = 0x7fffffff;
    for (int i = 0; i < 50; i++)
    {
        for (int j = 0; j < 50; j++)
        {
            if (map[i][j] == type)
            {
                des.push_back({i, j});
                if (manhatten_distance(x1, y1, i, j) < min_dis)
					min_dis = manhatten_distance(x1, y1, i, j);
            }
        } 
    }
    std::priority_queue<node> pq;
    pq.push({x1, y1, coordinate{-1, -1}, 0, (double)min_dis});
    memset(visited, false, sizeof(visited));
    while (!pq.empty())
    {
        node now = pq.top();
        pq.pop();
        if (visited[now.x][now.y])
            continue;
        visited[now.x][now.y] = true;
        from[now.x][now.y] = now.from;
        if (map[now.x][now.y] == type)
        {
            std::deque<coordinate> path;
            path.push_back({now.x, now.y});
            coordinate temp = now.from;
            while (temp.x != -1)
            {
                path.push_front({temp.x, temp.y});
                temp = from[temp.x][temp.y];
            }
            path.pop_front();
            return path;
        }
        auto is_empty = [](THUAI7::PlaceType t)
        {
            return t == THUAI7::PlaceType::Space or t == THUAI7::PlaceType::Shadow or t == THUAI7::PlaceType::Wormhole;
        };
        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {
                if (i == 0 and j == 0)
                    continue;
                if (i == -1 and j == -1 and (!is_empty(map[now.x - 1][now.y]) or !is_empty(map[now.x][now.y - 1])))
                    continue;
                if (i == 1 and j == -1 and (!is_empty(map[now.x + 1][now.y]) or !is_empty(map[now.x][now.y - 1])))
                    continue;
                if (i == -1 and j == 1 and (!is_empty(map[now.x - 1][now.y]) or !is_empty(map[now.x][now.y + 1])))
                    continue;
                if (i == 1 and j == 1 and (!is_empty(map[now.x + 1][now.y]) or !is_empty(map[now.x][now.y + 1])))
                    continue;
                int nx = now.x + i, ny = now.y + j;
                if (nx < 0 or nx >= 50 or ny < 0 or ny >= 50 or visited[nx][ny])
                    continue;
                if (map[nx][ny] == type)
                {
                    double cost = sqrt(i * i + j * j);
                    pq.push({nx, ny, coordinate{now.x, now.y}, now.cost + cost, 0});
                }
                THUAI7::PlaceType t = map[nx][ny];
                if (t != THUAI7::PlaceType::Space and t != THUAI7::PlaceType::Shadow and t != THUAI7::PlaceType::Wormhole)
                    continue;
                double cost = sqrt(i * i + j * j);
                min_dis = 0x7fffffff;
                for (const auto& d : des)
                {
                    if (manhatten_distance(nx, ny, d) < min_dis)
						min_dis = manhatten_distance(nx, ny, d);
                }
                pq.push({nx, ny, coordinate{now.x, now.y}, now.cost + cost, (double)min_dis});
            }
        }
    }
}

std::deque<coordinate> path;
bool find_resource = false;
bool getMap = false;
void AI::play(IShipAPI& api)
{
    if (!getMap)
    {
        auto m = api.GetFullMap();
        getMap = true;
        for (int i = 0; i < m.size(); i++)
        {
            for (int j = 0; j < m[i].size(); j++)
            {
				map[i][j] = m[i][j];
			}
            std::cout << std::endl;
		}
    }
    if (this->playerID == 1)
    {
        auto info = api.GetSelfInfo();
        int x = info->x, y = info->y;
        if (path.empty())
        {
            std::cout << "??????????????where is my path????????????" << std::endl;
            if (!find_resource)
            {
        	    path = search_road(x / 1000, y / 1000, THUAI7::PlaceType::Resource);
                find_resource = true;
            }
            else
            {
                path = search_road(x / 1000, y / 1000, THUAI7::PlaceType::Construction);
            }
            for (int i = 0; i < path.size(); i++)
            {
                std::cout << i << " x: " << path[i].x << " y: " << path[i].y << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::seconds(8));
        }
        if (api.GetSelfInfo()->shipState == THUAI7::ShipState::Idle)
        {
            auto next = path.front();
            path.pop_front();
            int dx = next.x * 1000 + 500 - x;
            int dy = next.y * 1000 + 500 - y;
            std::cout << "x: " << x << " y: " << y << "nx: " << next.x << "ny: " << next.y << std::endl;
            std::cout << "dx: " << dx << " dy: " << dy << std::endl;
            double time = sqrt(dx * dx + dy * dy) / 3.0;
            double angle = atan2(dy, dx);
            std::cout << "time: " << time << " angle: " << angle << std::endl;
		    api.Move(time, angle);
            //std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}
    }
    else if (this->playerID == 2)
    {
        while (true)
        {
            api.MoveDown(100);
            std::this_thread::sleep_for(std::chrono::seconds(1));
            api.MoveLeft(100);
            api.PrintSelfInfo();
            api.PrintShip();
            api.Recover(10);
            std::this_thread::sleep_for(std::chrono::seconds(1));
            api.Attack(1.5);
        }
    }

    else if (this->playerID == 3)
    {
    }
    else if (this->playerID == 4)
    {
        api.MoveDown(100);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        api.MoveLeft(100);
    }
}


//const char* get_placetype(THUAI7::PlaceType t);

void AI::play(ITeamAPI& api)  // 默认team playerID 为0
{
    //api.PrintSelfInfo();
    //api.InstallModule(2, THUAI7::ModuleType::ModuleLaserGun);
}

/*
        const char* get_placetype(THUAI7::PlaceType t)
{
    switch (t)
    {
        case THUAI7::PlaceType::NullPlaceType:
            return "nullplacetype";
        case THUAI7::PlaceType::Home:
            return "home";
        case THUAI7::PlaceType::Space:
            return "space";
        case THUAI7::PlaceType::Ruin:
            return "ruin";
            break;
        case THUAI7::PlaceType::Shadow:
            return "shadow";
            break;
        case THUAI7::PlaceType::Asteroid:
            return "asteroid";
        case THUAI7::PlaceType::Resource:
            return "resource";
        case THUAI7::PlaceType::Construction:
            return "construction";
            break;
        case THUAI7::PlaceType::Wormhole:
            return "wormhole";
        default:
            break;
    }
    return "?";
}*/

        /*
        if (!moved)
        {
             std::this_thread::sleep_for(std::chrono::seconds(6));
        double time = (y - 41.5 * 1000) / 3.0;
        api.MoveLeft(time);
        std::cout << time << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(int(time)));
        info = api.GetSelfInfo();
        std::cout << "x: " << info->x << " y: " << info->y << std::endl;
        time = (14 * 1000 - x) / 3.0;
        api.MoveDown(time);
        std::this_thread::sleep_for(std::chrono::milliseconds(int(time)+5));
        info = api.GetSelfInfo();
        std::cout << "x: " << info->x << " y: " << info->y << std::endl;
        time = 1000.0;
        api.MoveRight(time);
        std::this_thread::sleep_for(std::chrono::milliseconds(int(time)));
        info = api.GetSelfInfo();
        std::cout << "x: " << info->x << " y: " << info->y << std::endl;
        api.Produce();
        moved = true;
        }*/
