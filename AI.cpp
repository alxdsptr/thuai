#include <vector>
#include <thread>
#include <array>
#include <queue>
#include <list>
#include <unordered_set>
#include <cstring>
#include "AI.h"
#include "constants.h"
#define maxLen 55
// 注意不要使用conio.h，Windows.h等非标准库
// 为假则play()期间确保游戏状态不更新，为真则只保证游戏状态在调用相关方法时不更新，大致一帧更新一次
extern const bool asynchronous = false;
#ifndef PI
#define PI 3.14159265358979323846
#endif  // !PI


// 选手需要依次将player1到player4的船类型在这里定义
extern const std::array<THUAI7::ShipType, 4> ShipTypeDict = {
    THUAI7::ShipType::CivilianShip,
    THUAI7::ShipType::CivilianShip,
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
struct PointHash
{
    std::size_t operator()(const coordinate& c) const
    {
        return std::hash<int>()(c.x * 50 + c.y); 
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
    if (type == THUAI7::PlaceType::Wormhole)
        return 2;
}
std::unordered_set<coordinate, PointHash> des_list[4];

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
std::deque<coordinate> search_road(int x1, int y1, THUAI7::PlaceType type, IShipAPI& api)
{
    int min_dis = 0x7fffffff, index = getIndex(type);
    auto &des = des_list[index];
    for (auto i = des.begin(); i != des.end(); )
    {
        if (type == THUAI7::PlaceType::Resource)
        {
            std::cout << "x: " << (*i).x << " y: " << (*i).y << std::endl;
            int temp = api.GetResourceState((*i).x, (*i).y);
            if (temp == 0)
            {
                i = des.erase(i);
                continue;
            }
        }
        else if (type == THUAI7::PlaceType::Construction and api.GetConstructionHp((*i).x, (*i).y) > 0)
        {
            i = des.erase(i);
            continue;
        }
        int temp = manhatten_distance(x1, y1, *i);
        if (temp < min_dis)
            min_dis = temp;
        i++;
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
                if (des.find({nx,ny}) != des.end() and (i == 0 or j == 0))
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
            auto t = m[i][j];
            map[i][j] = t;
            if (t == THUAI7::PlaceType::Resource or t == THUAI7::PlaceType::Construction or t == THUAI7::PlaceType::Wormhole)
            {
                des_list[getIndex(m[i][j])].insert({i, j});
            }
            else if (t == THUAI7::PlaceType::Home)
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
bool judgeNear(int x, int y, THUAI7::PlaceType t)
{
    x = x / 1000, y = y / 1000;
    if (map[x - 1][y] == t or map[x][y - 1] == t or map[x + 1][y] == t or map[x][y + 1] == t)
    {
        return true;
    }
    return false;
};

void civilShip(IShipAPI& api)
{
    auto info = api.GetSelfInfo();
    int x = info->x, y = info->y;
        if (this_state == shipState::produce)
        {
            if (info->shipState == THUAI7::ShipState::Producing)
                return;
            if (path.empty())
            {
                if (judgeNear(x, y, THUAI7::PlaceType::Resource)){
                    int left = api.GetResourceState(target_pos.x, target_pos.y);
                    //std::cout << "distance: " << left << std::endl;
                    auto res = api.Produce();
                    bool success = res.get();
                    std::cout << (success ? "success" : "failed") << std::endl;
                    if (!success)
                    {
    	                path = search_road(x / 1000, y / 1000, THUAI7::PlaceType::Resource, api);
                    }
                }
                else
                {
    	            path = search_road(x / 1000, y / 1000, THUAI7::PlaceType::Resource, api);
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
                    api.Construct(construction_type);
                }
                else
                {
    	            path = search_road(x / 1000, y / 1000, THUAI7::PlaceType::Construction, api);
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
            std::cout << "back" << std::endl;
        }
    }
    if (this->playerID == 1)
    {
        civilShip(api);
    }
    else if (this->playerID == 2)
    {
        civilShip(api);
    }

    else if (this->playerID == 3)
    {
        auto info = api.GetSelfInfo();
        int x = info->x, y = info->y;
        if (path.empty())
        {
            path = search_road(x / 1000, y / 1000, THUAI7::PlaceType::Wormhole, api);
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
    else if (this->playerID == 4)
    {
        api.MoveDown(100);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        api.MoveLeft(100);
    }
}

enum NodeState
{
    IDLE = 0,
    SUCCESS,
    FAIL,
    RUNNING
};

//struct event
//{
//    bool finished = false;
//    std::function<bool(ITeamAPI& api)> condition;
//    std::function<NodeState(ITeamAPI& api)> action;
//    event(std::function<bool(ITeamAPI& api)> c, std::function<NodeState(ITeamAPI& api)> a) : condition(c), action(a) {}
//};



class rootNode
{
public:
    NodeState state;

    rootNode(NodeState x=IDLE) :
        state(x)
    {
    }

    virtual NodeState perform(ITeamAPI& api)
    {
        return IDLE;
    }

    virtual ~rootNode()
    {
    }
};

class eventNode :public rootNode
{
public:
    std::function<bool(ITeamAPI& api)> condition;
    std::function<NodeState(ITeamAPI& api)> action;
    eventNode(std::function<bool(ITeamAPI& api)> c, std::function<NodeState(ITeamAPI& api)> a) :
        condition(c),
        action(a)
    {
    }

    virtual NodeState perform(ITeamAPI& api)
    {
        switch (state)
        {
            case IDLE:
                state = RUNNING;
                break;
            case RUNNING:
                break;
            default:
                return state;
                break;
        }

        if (condition(api))
        {
            state = action(api);
        }
        else
        {
            state = FAIL;
        }

        return state;
    }


    virtual ~eventNode()
    {
    }
};

class SequenceNode :public rootNode
{
private:
    inline void RewindChildren()
    {
        for (size_t i = 0; i < events.size(); i++)
        {
            events[i]->state = IDLE;
        }
        curChild = 0;
    }

public:
    std::vector<rootNode *> events;
    int curChild;

    virtual NodeState perform(ITeamAPI& api)
    {
        switch (state)
        {
            case IDLE:
                state = RUNNING;
                break;
            case RUNNING:
                break;
            default:
                return state;
                break;
        }

        switch (events[curChild]->perform(api))
        {
            case RUNNING:
                break;
            case SUCCESS:
                if (curChild == events.size()-1)
                {
                    state = SUCCESS;
                    RewindChildren();
                }
                else
                {
                    curChild++;
                }
                break;
            case FAIL:
                state = FAIL;
                RewindChildren();
                break;
            default:
                break;
        }
        return state;
    }
    SequenceNode(std::initializer_list<rootNode *> l) : events(l),curChild(0) {}

    SequenceNode(const SequenceNode& a) :
        curChild(0)
    {
        for (size_t i = 0; i < a.events.size(); i++)
        {
            events.push_back(new auto(*a.events[i]));
        }
    }


    virtual ~SequenceNode()
    {
        for (size_t i = 0; i < events.size(); i++)
        {
            delete events[i];
            events[i] = NULL;
        }
    }
};

class TryUntilSuccessNode :public rootNode
{
public:
    rootNode* child;
    TryUntilSuccessNode(rootNode* x) :
        rootNode(RUNNING), child(x)
    {}

    virtual NodeState perform(ITeamAPI& api)
    {
        if (state==IDLE)
        {
            state = RUNNING;
        }
        else if (state==RUNNING)
        {
            switch (child->perform(api))
            {
                case SUCCESS:
                    state = SUCCESS;
                    child->state = IDLE;
                    break;
                case FAIL:
                    child->state = IDLE;
                    break;
                default:
                    break;
            }
        }
        return state;
    }

    virtual ~TryUntilSuccessNode()
    {
        delete child;
        child = NULL;
    }
};

class AlwaysSuccessNode : public rootNode
{
public:
    rootNode* child;
    AlwaysSuccessNode(rootNode* x) :
        rootNode(IDLE),
        child(x)
    {
    }

    virtual NodeState perform(ITeamAPI& api)
    {
        if (state == IDLE)
        {
            state = RUNNING;
        }
        if (state == RUNNING)
        {
            switch (child->perform(api))
            {
                case RUNNING:
                    state = RUNNING;
                    break;
                default:
                    state = SUCCESS;
                    break;
            }
        }
        return state;
    }

    virtual ~AlwaysSuccessNode()
    {
        delete child;
        child = NULL;
    }
};


//const char* get_placetype(THUAI7::PlaceType t);
bool hasSend = false;
bool hasInstall = false;
bool hasBuild = false;
bool BuildSecondCivil = false;
bool always(ITeamAPI&)
{
    return true;
}
auto SendMessage(int id, const std::string& m)
{
    return [=](ITeamAPI& api)
    {
        auto res = api.SendTextMessage(id, m);
        bool success = res.get();
        std::cout << "Send:" << success << std::endl;
        return success?SUCCESS:FAIL;
    };
}
auto EnergyThreshold(int threshold)
{
    return [threshold](ITeamAPI& api)
    { 
            return (api.GetEnergy() >= threshold); };
}
auto InstallModule(int id, THUAI7::ModuleType moduleType)
{
    return [=](ITeamAPI& api)
        {
		auto res = api.InstallModule(id, moduleType);
		bool success = res.get();
		return success?SUCCESS:FAIL;
	};
}
auto BuildShip(THUAI7::ShipType shipType)
{
    return [=](ITeamAPI& api)
        {
		auto res = api.BuildShip(shipType, 0);
		bool success = res.get();
		return success?SUCCESS:FAIL;
	};
}
//SequenceNode rootNode = {{always, SendMessage(1, "produce")}, 
//    {EnergyThreshold(8000), InstallModule(1, THUAI7::ModuleType::ModuleProducer3)}, 
//    {EnergyThreshold(12000), BuildShip(THUAI7::ShipType::MilitaryShip)}, 
//    {EnergyThreshold(4400), BuildShip(THUAI7::ShipType::CivilianShip)},
//    {EnergyThreshold(8000), InstallModule(2, THUAI7::ModuleType::ModuleProducer3)}};

SequenceNode root = {
    new AlwaysSuccessNode(new eventNode({always, SendMessage(1, "produce")})),
    new TryUntilSuccessNode(new eventNode({EnergyThreshold(8000), InstallModule(1, THUAI7::ModuleType::ModuleProducer3)})),
    new TryUntilSuccessNode(new eventNode({EnergyThreshold(4000), BuildShip(THUAI7::ShipType::CivilianShip)})),
    new TryUntilSuccessNode(new eventNode({EnergyThreshold(12000), BuildShip(THUAI7::ShipType::MilitaryShip)})),
    new TryUntilSuccessNode(new eventNode({EnergyThreshold(8000), InstallModule(2, THUAI7::ModuleType::ModuleProducer3)}))
};


void AI::play(ITeamAPI& api)  // 默认team playerID 为0
{
    //if (api.GetSelfInfo()->playerID)
    //{
    //    rootNode.events[0].action = SendMessage(2, "produce");
    //}
    root.perform(api);
    std::cout << api.GetSelfInfo()->teamID << "  "<<api.GetEnergy() << std::endl;
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
