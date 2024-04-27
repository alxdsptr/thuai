#include <vector>
#include <thread>
#include <array>
#include <queue>
#include <list>
#include <set>
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
    bool operator==(const coordinate& c) const
    {
        return x == c.x and y == c.y;
    }
    bool operator<(const coordinate& c) const
    {
        if (x < c.x)
			return true;
        if (x > c.x)
            return false;
        return y < c.y;
    }
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
int getIndex(THUAI7::PlaceType type)
{
    if (type == THUAI7::PlaceType::Resource)
        return 0;
    if (type == THUAI7::PlaceType::Construction)
        return 1;
}
std::set<coordinate> des_list[4];

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
    int min_dis = 0x7fffffff, index = getIndex(type);
    const auto &des = des_list[index];
    for (auto const& i : des)
    {
        int temp = manhatten_distance(x1, y1, i);
        if (temp < min_dis)
            min_dis = temp;
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
        if (des.find({now.x, now.y}) != des.end())
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
                for (auto const& k : des)
                {
                    int temp = manhatten_distance(x1, y1, k);
                    if (temp < min_dis)
                        min_dis = temp;
                }
                pq.push({nx, ny, coordinate{now.x, now.y}, now.cost + cost, (double)min_dis});
            }
        }
    }
}

std::deque<coordinate> path;
coordinate target_pos, home_pos, enemy_pos;
bool hasGetMap = false;
void getMap(IShipAPI& api, bool top)
{
    auto m = api.GetFullMap();
    for (int i = 0; i < m.size(); i++)
    {
        for (int j = 0; j < m[i].size(); j++)
        {
            map[i][j] = m[i][j];
            if (m[i][j] == THUAI7::PlaceType::Resource or m[i][j] == THUAI7::PlaceType::Construction)
            {
                des_list[getIndex(m[i][j])].insert({i, j});
            }
            else if (m[i][j] == THUAI7::PlaceType::Home)
            {
                if (i < 25)
                {
                    if(top) home_pos = {i, j};
                    else enemy_pos = {i, j};
                }
                else
                {
                    if(!top) home_pos = {i, j};
                    else enemy_pos = {i, j};
                }
            }
        }
    }
 }
enum class shipState
{
    produce,
    construct,
    back,
};
shipState this_state; //= shipState::produce;
THUAI7::ConstructionType construction_type;
void AI::play(IShipAPI& api)
{
    auto info = api.GetSelfInfo();
    int x = info->x, y = info->y;
    if (!hasGetMap)
    {
        hasGetMap = true;
        getMap(api, x/1000 < 25 ? true : false);
    }
    if (api.HaveMessage())
    {
        auto m = api.GetMessage();
        std::string message = m.second;
        if (message == "produce")
        {
            this_state = shipState::produce;
        }
        else if (message.find("construct") == 0)
        {
            this_state = shipState::construct;
            auto type = message.substr(9);
            if (type == "factory")
            {   
                construction_type = THUAI7::ConstructionType::Factory;
            }
            else if (type == "fort")
            {
                construction_type = THUAI7::ConstructionType::Fort;
            }
            else if (type == "community")
            {
                construction_type = THUAI7::ConstructionType::Community;
            }
        }
        else if (message.find("back") == 0)
        {
            this_state = shipState::back;
        }
    }
    api.GetGameInfo();
    auto judgeNear = [](int x, int y, THUAI7::PlaceType t)
    {
        x = x / 1000, y = y / 1000;
        if (map[x - 1][y] == t or map[x][y - 1] == t or map[x + 1][y] == t or map[x][y + 1] == t)
        {
            return true;
        }
        return false;
    };
    if (this->playerID == 1)
    {
        if (this_state == shipState::produce)
        {
            if (info->shipState == THUAI7::ShipState::Producing)
                return;
            if (path.empty())
            {
                if (judgeNear(x, y, THUAI7::PlaceType::Resource)){
                    int left = api.GetResourceState(target_pos.x, target_pos.y);
                    std::cout << "distance: " << left << std::endl;
                    if (left <= 10)
                    {
                        des_list[0].erase(target_pos);    
    	                path = search_road(x / 1000, y / 1000, THUAI7::PlaceType::Resource);
                        target_pos = path.back();
                        std::cout << "resource_pos:\n";
                        for (auto const& i : des_list[0])
                        {
							std::cout << i.x << " " << i.y << std::endl;
						}
                        std::cout << "path:\n";
                        for (auto const& i : path)
                        {
							std::cout << i.x << " " << i.y << std::endl;
						}
                    }
                    else
                    {
                        api.Produce();
                    }
                }
                else
                {
    	            path = search_road(x / 1000, y / 1000, THUAI7::PlaceType::Resource);
                    target_pos = path.back();
                }
            }
        }else if (this_state == shipState::construct)
        {
            if (info->shipState == THUAI7::ShipState::Constructing)
                return;
            if (path.empty())
            {
                if (judgeNear(x, y, THUAI7::PlaceType::Construction)){
                    des_list[1].erase(target_pos);
                    api.Construct(construction_type);
                }
                else
                {
    	            path = search_road(x / 1000, y / 1000, THUAI7::PlaceType::Construction);
                    target_pos = path.back();
                }
            }
        }
        else if (this_state == shipState::back)
        {
            if (path.empty())
            {
                path = search_road(x / 1000, y / 1000, home_pos.x, home_pos.y); 
                api.EndAllAction();
            }
        }
        if (api.GetSelfInfo()->shipState == THUAI7::ShipState::Idle and !path.empty())
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
        auto info = api.GetSelfInfo();
        int x = info->x, y = info->y;
        if (path.empty())
        {
            path = search_road(x / 1000, y / 1000, THUAI7::PlaceType::Wormhole);
        }
        if (api.GetSelfInfo()->shipState == THUAI7::ShipState::Idle and !path.empty())
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
bool hasSend = false;
bool hasInstall = false;
bool hasBuild = false;
void AI::play(ITeamAPI& api)  // 默认team playerID 为0
{
    //api.PrintSelfInfo();
    //api.InstallModule(2, THUAI7::ModuleType::ModuleLaserGun);
    if (!hasSend)
    {
        api.SendTextMessage(1, "produce");
        hasSend = true;
    }
    if (!hasInstall and api.GetEnergy() >= 7000)
    {
        api.SendTextMessage(1, "back");
        if (api.GetEnergy() >= 8000)
        {
            auto res = api.InstallModule(1, THUAI7::ModuleType::ModuleProducer3);
            bool success = res.get();
            std::cout << (success ? "success" : "failed") << std::endl;
            hasInstall = true;
            api.SendTextMessage(1, "produce");
        }
    }
    if (hasInstall and !hasBuild and api.GetEnergy() >= 8000)
    {
        api.BuildShip(THUAI7::ShipType::MilitaryShip, 0); 
        hasBuild = true;
    }

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
