#include <vector>
#include <thread>
#include <array>
#include <queue>
#include <stack>
#include <list>
#include <unordered_set>
#include <cstring>
#include "AI.h"
#include "constants.h"
#include<algorithm>

using std::cout, std::endl;
#define DEBUG
#define maxLen 55
// 注意不要使用conio.h，Windows.h等非标准库
// 为假则play()期间确保游戏状态不更新，为真则只保证游戏状态在调用相关方法时不更新，大致一帧更新一次
extern const bool asynchronous = false;
#ifndef PI
#define PI 3.14159265358979323846
#endif  // !PI

#define CIVILSHIP1 (0)
#define CIVILSHIP2 (1)
#define MILITARYSHIP (2)
#define FLAGSHIP (3)

#define SHIP_1 (1)
#define SHIP_2 (1 << 1)
#define SHIP_3 (1 << 2)
#define SHIP_4 (1 << 3)

#define MODEPARAM_ConstructFactory (1)
#define MODEPARAM_ConstructFort (2)
#define MODEPARAM_ConstructCommunity (3)

#define MODEPARAM_AttackHome (1)




#define Instruction_RefreshResource (1)
#define Instruction_RefreshConstruction (2)
#define Instruction_AttackState (3)
//#define Instruction_Inspecting (4)
#define WormholeDestroyed (4)
#define WormholeOpen (5)
#define Instruction_CounterAttack (6) 
#define Instruction_UnsetcounterAttack (7)

#define Parameter_ResourceRunningOut (1)
#define Parameter_ConstructionBuildUp (1)
#define Parameter_EnemyBuildConstruction (1<<1)
#define Parameter_DestroyedEnemyConstruction (1<<2)
#define Parameter_DestroyedFriendConstruction (1<<3)
#define Parameter_AttackSuccess (1<<4)

#define ShipStepID 0
#define DodgeID 1
#define RoadSearchID 2
#define ReturnHomeID 3
#define RecoveryID 4
#define InspectID 5

#define RATIO 1000

#define PRIORITY_Inspection (0.5)
#define PRIORITY_Recovery (4)
#define PRIORITY_ReturnHome (3)
#define PRIORITY_Dodge (2)
#define PRIORITY_Normal (0)

// 选手需要依次将player1到player4的船类型在这里定义
extern const std::array<THUAI7::ShipType, 4> ShipTypeDict = {
    THUAI7::ShipType::CivilianShip,
    THUAI7::ShipType::CivilianShip,
    THUAI7::ShipType::MilitaryShip,
    THUAI7::ShipType::MilitaryShip,
};
 
// 可以在AI.cpp内部声明变量与函数
enum Side
{
    RED = 0,
    BLUE
};

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
    coordinate(int x = 0, int y = 0) :
        x(x),
        y(y)
    {
    }

    bool operator!=(const coordinate& co) const
    {
        return !(*this == co);
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

// bool visited[maxLen][maxLen];
// coordinate from[maxLen][maxLen];


enum ShipMode
{
    IDLE = 0,
    ATTACK,
    REVENGE,
    HIDE,
    PRODUCE,
    CONSTRUCT,
    ROB,
    RUIN,
    FOLLOW,
    INSPECT
};

/**
 * @brief 为舰船行为函数提供返回值模板
 *
 */
struct ModeRetval
{
    ShipMode mode;
    bool immediate;
    size_t param;
};




/**
* 命名空间声明
*/

namespace Commute
{
    struct Buffer
    {
        ShipMode Mode = IDLE;
        char ModeParam;
        bool with_target;
        coordinate target;
        bool with_param = false;
        unsigned char instruction;
        unsigned char param;
        coordinate param_pos;
    };

    struct ReportBuffer
    {
        unsigned char instruction;
        unsigned char param;
        coordinate param_pos;
    };

    constexpr size_t BUFFER_SIZE = sizeof(Commute::Buffer);
    constexpr size_t REPORTBUFFER_SIZE = sizeof(Commute::ReportBuffer);
    using bytePointer = unsigned char*;
}
namespace HomeInfo
{

    bool UsableShip[5] = {0}, appearedId[5] = {0};
    int MyScore = 0;
    int EnemyScore = 0;
    int MyMoney = 0;
    int EnemyMoney = 0;
    int EnemyConsume = 0;
    int EnemyGain = 0;
    Side MySide = RED;

    coordinate MyHomePos, EnemyHomePos;

    // 为什么不用shared_ptr<const THUAI7::Ship>?
    std::vector<THUAI7::Ship> EnemyShips, EnemyShipsInSight;
    THUAI7::Ship MyShips[4];
    int index_to_id[4], ship_cnt = 0;

    /**
     * @brief 给舰船发送信息的缓冲区
     * 下标为playerID
     */
    Commute::Buffer TeamShipBuffer[5];

    int init = 1;


    void CheckInfo(ITeamAPI& api);
}  // namespace HomeInfo

namespace MapInfo
{
    enum Place
    {
        NullPlaceType = 0,
        Space,
        Ruin,
        Shadow,
        Asteroid,
        Resource,
        Construction,
        MyHome,
        EnemyHome,
        OpenWormhole,
        ClosedWormhole
    };

    /**
    * @brief 储存各类型格子的坐标的向量，其中资源只有在可能有剩余量的时候才会储存在其中
    */
    std::unordered_set<coordinate, PointHash> PositionLists[11];
    std::unordered_set<coordinate, PointHash> resource[2], construction[2];
    std::vector<coordinate> myResAndCons, enemyResAndCons;

    std::unordered_set<coordinate, PointHash> Ori_PositionLists[11];


 //   std::unordered_set<coordinate, PointHash> NoStep;


    /**
     * @brief 储存虫洞状态，true为可通行
     */
    bool WormHolestat[3] = {false, true, false};


    Place fullmap[50][50];


    THUAI7::PlaceType map[maxLen][maxLen];


//    std::unordered_set<coordinate, PointHash> des_list[4];
    coordinate home_pos, enemy_pos;
    bool hasGetMap = false;
    Side MySide;


    int convertSide(int x, int y)
    {
        if ((x < 25 and MySide == RED) or (x > 25 and MySide == BLUE))
            return 1;
        else
            return 0;
    }
    Place PlaceTypeConvert(THUAI7::PlaceType t, int x, int y)
    {
        switch (t)
        {
            case THUAI7::PlaceType::NullPlaceType:
                return NullPlaceType;
            case THUAI7::PlaceType::Home:
                if ((x < 10 and MySide == RED) or (x >= 40 and MySide == BLUE))
					return MyHome;
				else
					return EnemyHome;
            case THUAI7::PlaceType::Space:
                return Space;
            case THUAI7::PlaceType::Ruin:
                return Ruin;
            case THUAI7::PlaceType::Shadow:
                return Shadow;
            case THUAI7::PlaceType::Asteroid:
                return Asteroid;
            case THUAI7::PlaceType::Resource:
                resource[convertSide(x, y)].insert(coordinate(x, y));
                return Resource;
            case THUAI7::PlaceType::Construction:
                construction[convertSide(x, y)].insert(coordinate(x, y));
                return Construction;
            case THUAI7::PlaceType::Wormhole:
                if(y <= 15){
//                    wormholes[0].push_back({x, y});
                    return ClosedWormhole;
                }else if(y >= 35){
//                    wormholes[2].push_back({x, y});
                    return ClosedWormhole;
                }else
                {
//                    wormholes[1].push_back({x, y});
                    return OpenWormhole;
                }
            default:
                break;
        }
    }
    int getIndex(THUAI7::PlaceType type)
    {
        if (type == THUAI7::PlaceType::Resource)
            return 0;
        if (type == THUAI7::PlaceType::Construction)
            return 1;
        if (type == THUAI7::PlaceType::Wormhole)
            return 2;
    }
    int resource_cnt;
    template<class T>
    void LoadFullMap(T& api)
    {
        auto mp = api.GetFullMap();
        auto self = api.GetSelfInfo();
        self->teamID == 0 ? MySide = RED : MySide = BLUE;
        for (size_t i = 0; i < 50; i++)
        {
            for (size_t j = 0; j < 50; j++)
            {
                map[i][j] = mp[i][j];
                fullmap[i][j] = PlaceTypeConvert(mp[i][j], i, j);
                //PositionLists[PlaceTypeConvert(mp[i][j])].insert(coordinate(i, j));
                PositionLists[fullmap[i][j]].insert(coordinate(i, j));
                Ori_PositionLists[fullmap[i][j]].insert(coordinate(i, j));
            }
        }
        resource_cnt = PositionLists[Resource].size();
        for (const auto& p : resource[1])
        {
            myResAndCons.push_back(p);
        }
        for (const auto& p : construction[1])
        {
            myResAndCons.push_back(p);
        }
        for (const auto& p : resource[0])
        {
            enemyResAndCons.push_back(p);
        }
        for (const auto& p : construction[0])
        {
            enemyResAndCons.push_back(p);
        }
        //#ifdef DEBUG
//        for(int i = 0; i < 3; i++){
//            std::cout << "Wormhole " << i << " : " << std::endl;
//            for(auto &p : wormholes[i]){
//                std::cout << p.x << " " << p.y << std::endl;
//            }
//        }
//#endif
    }
    void eraseResource(coordinate p)
    {
		resource[convertSide(p.x, p.y)].erase(p);
		PositionLists[Resource].erase(p);
	}
    void eraseConstruction(coordinate p)
    {
        construction[convertSide(p.x, p.y)].erase(p);
        PositionLists[Construction].erase(p);
    }
    void insertResource(coordinate p)
    {
		resource[convertSide(p.x, p.y)].insert(p);
		PositionLists[Resource].insert(p);
	}
    void insertConstruction(coordinate p)
    {
		construction[convertSide(p.x, p.y)].insert(p);
		PositionLists[Construction].insert(p);
	}

}  // namespace MapInfo

const std::vector<coordinate> wormholes[3] =
    {{{23, 10}, {23, 11}, {24, 10}, {24, 11}, {25, 10}, {25, 11}, {26, 10}, {26, 11}},
     {{23, 24}, {23, 25}, {24, 24}, {24, 25}, {25, 24}, {25, 25}, {26, 24}, {26, 25}},
     {{23, 38}, {23, 39}, {24, 38}, {24, 39}, {25, 38}, {25, 39}, {26, 38}, {26, 39}}};
int getWormholeIndex(int x, int y)
{
    if (y <= 15)
        return 0;
    if (y >= 35)
        return 2;
    return 1;
}
int getWormholeIndex(coordinate temp)
{
    if (temp.y <= 15)
        return 0;
    if (temp.y >= 35)
        return 2;
    return 1;
}
void openWormhole(coordinate tmp)
{
    int index = getWormholeIndex(tmp);
    for (auto const& i : wormholes[index])
    {
        //MapInfo::PositionLists[MapInfo::OpenWormhole].insert(i);
        //MapInfo::PositionLists[MapInfo::ClosedWormhole].erase(i);
        MapInfo::fullmap[i.x][i.y] = MapInfo::OpenWormhole;
    }
}
void closeWormhole(coordinate tmp)
{
    int index = getWormholeIndex(tmp);
    for (auto const& i : wormholes[index])
    {
        //MapInfo::PositionLists[MapInfo::ClosedWormhole].insert(i);
        //MapInfo::PositionLists[MapInfo::OpenWormhole].erase(i);
        MapInfo::fullmap[i.x][i.y] = MapInfo::ClosedWormhole;
    }
}
namespace ShipInfo
{
    struct ShipMem
    {
        // THUAI7::Ship NearestEnemyShip;
        ShipMode mode=IDLE;
        THUAI7::Ship me;
        // THUAI7::ConstructionType ConsType;
        // const char end = '\0';
    };

    /**
    * @brief 自身的信息
    * @param mode ShipMode
    * @param myself THUAI7::Ship
    * @param TargetPos coordinate
    */
    ShipMem myself;



    /**
    * @brief 敌舰
    */
    std::vector<std::shared_ptr<const THUAI7::Ship>> Enemies;

    /**
     * @brief 我方舰船
     */
    std::vector<std::shared_ptr<const THUAI7::Ship>> FriendShips;

    bool nearEnemy = false;
    bool nearBullet = false;
    std::vector<std::shared_ptr<const THUAI7::Bullet>> Bullets;
    /**
    * @brief 最近的敌舰
    */
    THUAI7::Ship NearestEnemy;

    /**
     * @brief 信息接收缓冲区
     */
    Commute::Buffer ShipBuffer;
    Commute::ReportBuffer ReportBuffer;

    THUAI7::ConstructionType constructType = THUAI7::ConstructionType::Factory;


    /**
     * @brief 是否攻击虫洞
     */
    bool WhetherAttackWormhole = 1;



    /**
     * @brief 是否是初始化状态
     */
    int init = 1;
    /**
     * @brief 获取自身当前状态，并储存附近的敌人
     * @param api 
     */
    void CheckInfo(IShipAPI& api)
    {
        myself.me = *api.GetSelfInfo();
        auto enem = api.GetEnemyShips();
        
        auto bullets = api.GetBullets();

        Enemies = std::move(enem);
        Bullets = std::move(bullets);

        auto fri = api.GetShips();
        FriendShips.clear();
        for (auto const& ship : fri)
        {
            if (ship->playerID == myself.me.playerID)
                continue;
            FriendShips.push_back(ship);
        }


        //Enemies.clear();
        if (!Enemies.empty())
        {
            nearEnemy = true;
            //            NearestEnemy = *enem[0];
//            Enemies.push_back(*enem[0]);
//            for (size_t i = 1; i < enem.size(); i++)
//            {
//                Enemies.push_back(*enem[i]);
//                if (manhatten_distance(enem[i]->x, enem[i]->y, myself.me.x, myself.me.y) < manhatten_distance(NearestEnemy.x, NearestEnemy.y, myself.me.x, myself.me.y))
//                {
//                    NearestEnemy = *enem[i];
//                }
//            }
        }
        if (!Bullets.empty())
        {
            nearBullet = true; 
        }
        if (init)
        {
            MapInfo::LoadFullMap(api);
            init = 0;
        }
    }



    bool InMyHalfPlace()
    {
        return true;
    }

}


/**
 * @brief 判断自己是否在某类型格子附近
 * @param x x坐标，注意不是格子数
 * @param y y坐标，注意不是格子数
 * @param t 格子类型
 * @return 
 */
bool judgeNear(int x, int y, THUAI7::PlaceType t)
{
    x = x / 1000, y = y / 1000;
    if (MapInfo::map[x - 1][y] == t or MapInfo::map[x][y - 1] == t or MapInfo::map[x + 1][y] == t or MapInfo::map[x][y + 1] == t)
    {
        return true;
    }
    return false;
}

inline int manhatten_distance(int x1, int y1, int x2, int y2)
{
    return abs(x1 - x2) + abs(y1 - y2);
}
inline int manhatten_distance(int x1, int y1, coordinate c)
{
    return abs(x1 - c.x) + abs(y1 - c.y);
}
inline double euclidean_distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}
inline double euclidean_distance(coordinate x, coordinate y)
{
    return sqrt((x.x - y.x) * (x.x - y.x) + (x.y - y.y) * (x.y - y.y));
}
double WeaponDis[6] = {0, 4000, 4000, 4000, 6000, 6000};
double WeaponToDis(THUAI7::WeaponType p)
{
    return WeaponDis[(int)p];
}
bool can_walk(int x, int y);
bool can_attack(int x, int y);
bool can_see(int x, int y);


/**
 * @brief 加入了判断有无遮挡物功能的攻击函数（注意，不保证一定在射程内，仅检测到目标的途中有无障碍）
 * @param api 
 * @param angle 攻击的角度
 * @param target 攻击的目标（格子数）
 * @return (-1,-1)表示正常运行；(-2,-2)表示找不到路径；若不是，则返回的是最近的可以攻击到目标的坐标
 */
inline coordinate MyAttack(IShipAPI& api, double angle, coordinate target)
{
    double x = ShipInfo::myself.me.x;
    double y = ShipInfo::myself.me.y;

    coordinate tar(target.x * 1000 + 500, target.y * 1000 + 500);

    double fulldistance = euclidean_distance(ShipInfo::myself.me.x, ShipInfo::myself.me.y, tar.x,tar.y);

    std::cout << "ENter MyATTACK\n";

    /**
    * @brief
    * y
    * ↑
    * ②--④
    * |  |
    * ①--③ →x
    */
    //coordinate Gridpoint[4];

    auto judge_ok = [&]() {
        bool ok = true;
        static int r = 200;
        for (double i = 400; i < fulldistance and ok; i += 400)
        {
            int x1 = x + cos(angle) * i;
            int y1 = y + sin(angle) * i;
            int x0 = (x1 / 1000) * 1000;
            int y0 = (y1 / 1000) * 1000;
            int lx = x1 - x0;
            int ly = y1 - y0;
            std::vector<coordinate> judge;

            coordinate tmp(x1 / 1000, y1 / 1000);  // 格子数的坐标
            THUAI7::PlaceType tmptype = MapInfo::map[tmp.x][tmp.y];
            if (tmp==target)
            {
                break;
            }
            if (tmp != target and !can_attack(tmp.x, tmp.y))
                //(tmptype == THUAI7::PlaceType::Asteroid || tmptype == THUAI7::PlaceType::Construction || tmptype == THUAI7::PlaceType::Home || tmptype == THUAI7::PlaceType::Resource || tmptype == THUAI7::PlaceType::Ruin || (MapInfo::PositionLists[MapInfo::ClosedWormhole].find(tmp) != MapInfo::PositionLists[MapInfo::ClosedWormhole].end())))
            {
                return false;
            }
            if (lx < r)
            {
                judge.push_back({tmp.x - 1, tmp.y});
            }
            else if (lx > 1000 - r)
			{
				judge.push_back({tmp.x + 1, tmp.y});
			}
            if (ly < r)
            {
                judge.push_back({tmp.x, tmp.y - 1});
			}
			else if (ly > 1000 - r){
                judge.push_back({tmp.x, tmp.y + 1});
            }
            if (euclidean_distance(x1, y1, x0, y0) < r)
			{
				judge.push_back({tmp.x - 1, tmp.y - 1});
			}
            else if (euclidean_distance(x1, y1, x0 + 1000, y0) < r)
            {
				judge.push_back({tmp.x + 1, tmp.y - 1});
            }
            else if (euclidean_distance(x1, y1, x0 + 1000, y0 + 1000) < r)
            {
				judge.push_back({tmp.x + 1, tmp.y + 1});
            }
            else if (euclidean_distance(x1, y1, x0, y0 + 1000) < r)
            {
				judge.push_back({tmp.x - 1, tmp.y + 1});
            }
            for (auto &i : judge)
			{
                if (i.x < 0 or i.x >= 50 or i.y < 0 or i.y >= 50)
				{
					continue;
				}
				if (i != target and !can_attack(i.x, i.y))
				{
					return false;
				}
			}


            /*
            int cnt = 0;
            // 储存四个格点的坐标
            for (int j = 0; j < 2; j++)
            {
                for (int k = 0; k < 2; k++)
                {
                    Gridpoint[cnt] = {(int)x1 / 1000 * 1000 + 1000 * j, (int)y1 / 1000 * 1000 + 1000 * k};
                    cnt++;
                }
            }

            // 判断格点
            for (int i = 0; i < 4 && ok; i++)
            {
                if (euclidean_distance(Gridpoint[i].x, Gridpoint[i].y, x1, y1) < 200)
                {
                    // 判断四个格子
                    for (int j = 0; j < 2; j++)
                    {
                        for (int k = 0; k < 2; k++)
                        {
                            coordinate tmp((((int)Gridpoint[i].x) - 500 + 1000 * j) / 1000, (((int)Gridpoint[i].y) - 500 + 1000 * k) / 1000);  // 格子数的坐标
                            THUAI7::PlaceType tmptype = MapInfo::map[tmp.x][tmp.y];
                            if (tmp != target and !can_walk(tmp.x, tmp.y))
                            {
                                return false;
                            }
                        }
                    }
                }
            }*/
        }
        return true;
    };

    if (judge_ok())
    {
        std::cout << "ENter MyATTACK: Judge OK!!!\n";

        api.Attack(angle);
        return {-1, -1};
    }
    std::cout << "ENter MyATTACK: NOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOPPPPPPPPPPPPPPPPPPPPPEEEEEEEEEEEEEEEE\n";

    double tmp_ang = angle;
    for (angle = tmp_ang - PI / 2; angle < tmp_ang + PI / 2 ; angle += PI / 16)
    {
        if (judge_ok())
        {
            coordinate tmp = {(int)(tar.x - cos(angle) * fulldistance) / 1000, (int)(tar.y - sin(angle) * fulldistance) / 1000};
            MapInfo::Place pltye = MapInfo::fullmap[tmp.x][tmp.y];
            std::cout << "MyAttack detected an available place, target=" << tmp.x << "  " << tmp.y << "\n";
            if (pltye==MapInfo::Shadow||pltye==MapInfo::Space)
            {
                return tmp;
            }
        }
    }

    return {-2, -2};

}




namespace BT
{

    /**
     * @enum NodeState
     * @brief 节点运行状态
     */
    enum NodeState
    {
        IDLE = 0,  ///< 未初始化值
        SUCCESS,   ///< 成功
        FAIL,      ///< 失败
        RUNNING    ///< 操作尚未完成，需要继续运行该节点
    };

    /**
     * @class eventNode
     * @brief 事件节点，当条件为真执行事件，并储存事件执行状态；否则节点状态为FAIL
     * @param condition 返回是否执行的布尔值的条件函数
     * @param action 待执行的操作对应的函数
     *
     */
    template <typename T>
    class eventNode
    {
    public:
        NodeState state = IDLE;
        std::function<bool(T& api)> condition;    ///< 执行条件
        std::optional<std::function<bool(T& api)>> judgeSuccess;  ///< 补充用来判断是否成功的函数
        std::function<NodeState(T& api)> action;  ///< 待执行的函数
        eventNode(std::function<bool(T& api)> c, std::function<NodeState(T& api)> a) :
            state(IDLE),
            condition(c),
            judgeSuccess(std::nullopt),
            action(a)
        {}
        eventNode(std::function<bool(T& api)> c, std::function<NodeState(T& api)> a, std::function<bool(T& api)> jf):
            state(IDLE),
            condition(c),
            judgeSuccess(jf),
            action(std::move(a))
        {}

        NodeState perform(T& api)
        {
//            switch (state)
//            {
//                case IDLE:
//                    state = RUNNING;
//                    break;
//                case RUNNING:
//                    break;
//                default:
//                    return state;
//                    break;
//            }

            if (condition(api))
            {
                state = action(api);
            }
            else if (judgeSuccess.has_value() and judgeSuccess.value()(api))
            {
				state = SUCCESS;
            }
            else
            {
                state = FAIL;
            }

            return state;
        }

        ~eventNode() = default;

    };
    /**
     * @class baseNode
     * @brief 提供行为树节点的基本内容
     * @see NodeState
     */
    template<typename T>
    class baseNode
    {
    public:
        NodeState state;

        baseNode(NodeState x = IDLE) :
            state(x)
        {
        }
        virtual NodeState perform(T& api) = 0;
        virtual ~baseNode()= default;
    };


    /**
     * @class SequenceNode
     * @brief 队列节点，依次执行子节点，直到节点返回结果为FAIL或全部子节点都被执行完
     * @param state 与最后一个执行的子节点的状态相同
     */
    template<typename T>
    class SequenceNode : public baseNode<T>
    {
    private:
        inline void ResetChildren()
        {
            for (size_t i = 0; i < events.size(); i++)
            {
                events[i]->state = IDLE;
            }
            curChild = 0;
        }

    public:
        std::vector<eventNode<T>*> events;
        int curChild = 0;
        //NodeState state;

        /**
         * @brief 队列节点的perform函数\n
         * - 按照顺序执行子节点，如果成功，则下次调用时执行下一节点（如果全部执行完，将重置子节点状态）；如果失败，本队列节点的状态将被设为FAIL，并重置子节点状态
         * @param api
         * @return 仅当当前执行的子节点返回FAIL时返回FAIL、最后一个子节点返回SUCCESS时返回SUCCESS；否则返回RUNNING
         */
        NodeState perform(T& api) override
        {
//            switch (state)
//            {
//                case IDLE:
//                    state = RUNNING;
//                    break;
//                case RUNNING:
//                    break;
//                default:
//                    return state;
//                    break;
//            }
            baseNode<T>::state = RUNNING;
            switch (events[curChild]->perform(api))
            {
                case RUNNING:
                    break;
                case SUCCESS:
                    if (curChild == events.size() - 1)
                    {
                        baseNode<T>::state = SUCCESS;
                        ResetChildren();
                    }
                    else
                    {
                        curChild++;
                    }
                    break;
                case FAIL:
                    baseNode<T>::state = FAIL;
//                    ResetChildren();
                    break;
                default:
                    break;
            }
            std::cout << curChild << std::endl;
            return baseNode<T>::state;
        }
        SequenceNode(std::initializer_list<eventNode<T>*> l) :
            events(l),
            curChild(0)
        {
        }
        void operator=(std::initializer_list<eventNode<T>*> l)
        {
            events = l;
            curChild = 0;
        }
        SequenceNode() :
            curChild(0)
        {
		}

        SequenceNode(const SequenceNode& a) :
            curChild(0)
        {
            for (size_t i = 0; i < a.events.size(); i++)
            {
                events.push_back(new auto(*a.events[i]));
            }
        }

        ~SequenceNode() override
        {
            for (size_t i = 0; i < events.size(); i++)
            {
                delete events[i];
                events[i] = nullptr;
            }
        }
    };
}


namespace ShipFSM
{

    class State
    {
    public:
        std::function<bool(IShipAPI& api)> action;

        std::function<State*(IShipAPI& api)> transform;

        State* Invoke(IShipAPI& api)
        {
            action(api);
            return transform(api);
        }
    };

    class fsm
    {
    public:
        State * curstat;
        //std::vector<State*> stat_list;

        void Invoke(IShipAPI & api)
        {
            curstat = curstat->Invoke(api);
        }
    };



}

namespace Conditions
{
    bool always(ITeamAPI&)
    {
        return true;
    }
    bool always_ship(IShipAPI&)
    {
        return true; 
    }
    auto EnergyThreshold(int threshold)
    {
        return [threshold](ITeamAPI& api)
        { return (api.GetEnergy() >= threshold); };
    }
    /*
    auto ShipNumThreshold(int threshold)
    {
		return [threshold](ITeamAPI& api)
        { return (HomeInfo::MyShips.size() >= threshold); };
	}*/
    auto ShipAvailable(int shipID)
    {
        return [=](ITeamAPI& api)
            {
			return HomeInfo::UsableShip[shipID];
		};
	}
    auto ShipHasProducer(int shipID, THUAI7::ProducerType type)
    {
        return [=](ITeamAPI& api) {
            return HomeInfo::MyShips[shipID - 1].producerType == type;
        };
    }
    auto ShipHasConstructor(int shipID, THUAI7::ConstructorType type)
    {
        return [=](ITeamAPI& api)
        {
            return HomeInfo::MyShips[shipID - 1].constructorType == type;
        };
    }
    auto ShipHasArmor(int shipID, THUAI7::ArmorType type)
    {
        return [=](ITeamAPI& api)
        {
            return HomeInfo::MyShips[shipID - 1].armorType == type;
        };
    }
    auto ShipHasShield(int shipID, THUAI7::ShieldType type)
    {
        return [=](ITeamAPI& api)
        {
            return HomeInfo::MyShips[shipID - 1].shieldType == type;
        };
    }
    auto ShipHasWeapon(int shipID, THUAI7::WeaponType type)
    {
        return [=](ITeamAPI& api)
        {
            return HomeInfo::MyShips[shipID - 1].weaponType == type;
        };
    }
    auto JudgeSelfState(THUAI7::ShipState state)
    {
        return [=](IShipAPI& api) {
            return ShipInfo::myself.me.shipState == state;
        };
    }
    auto PreAttack()
    {
        return [](IShipAPI& api) {
            return ShipInfo::myself.me.shipState == THUAI7::ShipState::Idle;
        };
    }
}  
namespace ShipAction
{
    class MoveFunc
    {
    private:
        double angle;
        int time;
    public:
        void set(double a, int b)
        {
            angle = a, time = b; 
        }
        MoveFunc(double a, int b) :
            angle{a},
            time{b}
        {}
        BT::NodeState operator()(IShipAPI& api)
        {
            api.Move(time, angle);
            return BT::NodeState::SUCCESS;
        }
    };
    class AttackFunc
    {
    private:
        double target_x, target_y;
    public:
        void set(double b, double c)
        {
            target_x = b, target_y = c;
        }
        AttackFunc(double b, double c) :
            target_x{b},
            target_y{c}
        {}
        BT::NodeState operator()(IShipAPI& api)
        {
            auto me = api.GetSelfInfo();
            double x = me->x, y = me->y;
            double angle = atan2(target_y - y, target_x - x);
            api.Attack(angle);
            return BT::NodeState::SUCCESS;
        }
    };
    /*
    BT::SequenceNode<IShipAPI> DodgeAndAttack = {
        new BT::eventNode<IShipAPI>(Conditions::always, MoveFunc{0, 0}),
        new BT::eventNode<IShipAPI>{Conditions::JudgeSelfState(THUAI7::ShipState::Idle), AttackFunc(0, 0)}
    };*/
    /*
    class DodgeNode
    {
        bool moveState = false;
        //bool attackState = false;
        std::function<bool(IShipAPI&)> cond = Conditions::JudgeSelfState(THUAI7::ShipState::Idle);

    public:
        MoveFunc move{0, 0};
        AttackFunc attack{0, 0};
        bool perform(IShipAPI& api)
        {
            if (!moveState)
            {
                move(api);
                moveState = true;
            }else if (moveState and cond(api)){
                attack(api);
                return true;
            }
            return false;
        }
    }dodge;*/
}

namespace HomeAction
{

    template<int id>
    auto SendMessage(const std::string& m)
    {
        return [=](ITeamAPI& api)
        {
            auto res = api.SendTextMessage(id, m);
            bool success = res.get();
            return success ? BT::SUCCESS : BT::FAIL;
        };
    }

    auto InstallModule(int id, THUAI7::ModuleType moduleType)
    {
        return [=](ITeamAPI& api)
        {
            auto res = api.InstallModule(id, moduleType);
            bool success = res.get();
            return success ? BT::SUCCESS : BT::FAIL;
        };
    }
    auto BuildShip(THUAI7::ShipType shipType)
    {
        return [=](ITeamAPI& api)
        {
            auto res = api.BuildShip(shipType, 0);
            bool success = res.get();
            return success ? BT::SUCCESS : BT::FAIL;
        };
    }


    coordinate tmp_coordinate = {-1, -1};
    /**
     * @brief 设置（多艘）舰船状态
     * @param ShipID 支持按位或来同时设置多个舰船的状态
     * @param mode 
     * @return 
     */
     class SetShipMode{
         unsigned char shipID;
         ShipMode mode;
         coordinate& target;
         unsigned char ModeParam;
     public:
         SetShipMode(unsigned char ShipID, ShipMode mode, unsigned char ModeParam = 0, coordinate& target = tmp_coordinate) :
             shipID(ShipID),
             mode(mode),
             target(target),
             ModeParam(ModeParam)
         {
         }

         BT::NodeState operator()(ITeamAPI& api)
         {
             int count;
             unsigned char id = shipID;
             for (count = 0; id; count++, id >>= 1)
             {
                 int tmp = id & 1;
                 if (tmp)
                 {
                     HomeInfo::TeamShipBuffer[HomeInfo::index_to_id[count]].Mode = mode;
                     if (target.x>=0)
                     {
                         HomeInfo::TeamShipBuffer[HomeInfo::index_to_id[count]].with_target = true;
                         HomeInfo::TeamShipBuffer[HomeInfo::index_to_id[count]].target = target;
                     }

                    HomeInfo::TeamShipBuffer[HomeInfo::index_to_id[count]].ModeParam = ModeParam;


                 }
             }
             return BT::SUCCESS;
         }
     };
}



namespace revenge
{

    inline double min(double a, double b)
    {
        return a > b ? b : a;
    }

}

namespace HomeInfo
{
    int first_id;
    std::vector<int> reviveList;
    void CheckInfo(ITeamAPI& api)
    {
        //初始化地图并确定我方颜色
        if (init)
        {
            MapInfo::LoadFullMap(api);
            MySide = (api.GetSelfInfo()->teamID == 0) ? RED : BLUE;
            /*
            for (size_t i = 1; i <= 4; i++)
            {
                TeamShipBuffer[i].Mode = IDLE;
            }*/
        }

        //刷新我方舰船及我方、对方基地坐标
        {
            auto sps = api.GetShips();
            //memset(UsableShip, 0, 4 * sizeof(int));
            bool temp[5] = {0};
            for (size_t i = 0; i < sps.size(); i++)
            {
                int id = sps[i]->playerID;
                if (init)
                {
                    first_id = id;
                }
                MyShips[id - 1] = *sps[i];
                if (UsableShip[id] == false and !appearedId[id])
                {
                    index_to_id[ship_cnt] = id;
                    appearedId[id] = true;
                    ship_cnt++;
                }
                temp[id] = true;
            }
            for (int i = 1; i <= 4; i++)
            {
                if (UsableShip[i] and !temp[i])
                {
                    reviveList.push_back(i);
                }
                UsableShip[i] = temp[i];
            }
                /*
            if (init)
            {
                coordinate tmp(MyShips[0].x / 1000, MyShips[0].y/1000);
                for (size_t i = 0; i < 2; i++)
                {
                    if (manhatten_distance(tmp.x,tmp.y,MapInfo::PositionLists[MapInfo::Home][i])<=2)
                    {
                        MyHomePos = MapInfo::PositionLists[MapInfo::Home][i];
                        EnemyHomePos = MapInfo::PositionLists[MapInfo::Home][1 - i];
                        break;
                    }
                }
            }*/
        }

        //刷新敌方舰船
        {
            auto sps = api.GetEnemyShips();
            EnemyShipsInSight.clear();
            for (size_t i = 0; i < sps.size(); i++)
            {
                EnemyShipsInSight.push_back(*sps[i]);
                bool add = true;
                for (size_t j = 0; j < EnemyShips.size() && add; j++)
                {
                    if (sps[i]->playerID == EnemyShips[j].playerID)
                    {
                        EnemyShips[j] = *sps[i];
                        add = false;
                    }
                }
                EnemyShips.push_back(*sps[i]);
            }
        }

        //刷新双方经济数据
        {
        MyMoney = api.GetEnergy();
        auto gminfo = api.GetGameInfo();
        
        int newscore, newenergy;
        if (MySide==RED)
        {
            MyScore = gminfo->redScore;
            MyMoney = gminfo->redEnergy;
            newscore = gminfo->blueScore;
            newenergy = gminfo->blueEnergy;
            if (init)
            {
                EnemyConsume = EnemyGain = 0;
            }
            else
            {
                EnemyGain = newscore - EnemyScore;
                EnemyConsume = EnemyMoney - newenergy - EnemyGain;
            }
            EnemyMoney = newenergy;
            EnemyScore = newscore;
        }
        else
        {
            MyScore = gminfo->blueScore;
            MyMoney = gminfo->blueEnergy;
            newscore = gminfo->redScore;
            newenergy = gminfo->redEnergy;
            if (init)
            {
                EnemyConsume = EnemyGain = 0;
            }
            else
            {
                EnemyGain = newscore - EnemyScore;
                EnemyConsume = EnemyMoney - newenergy - EnemyGain;
            }
            EnemyMoney = newenergy;
            EnemyScore = newscore;
        }
        }




        init = 0;
    }
}



namespace Commute
{
    bool RefreshInfo(IShipAPI& api)
    {
        if (!api.HaveMessage())
        {
            return false;
        }
        else
        {
            std::string mes = (api.GetMessage()).second;
            memcpy(&ShipInfo::ShipBuffer, mes.data(), sizeof(Buffer));
            return true;
        }
    }
    void sync_ships(ITeamAPI& api)
    {
        //for (size_t i = 0; i < HomeInfo::MyShips.size(); i++)
        for (auto const& ship : HomeInfo::MyShips)
        {
            int index = ship.playerID;
            if (!HomeInfo::UsableShip[index])
            {
				continue;
			}
            std::string message;
            message.resize(BUFFER_SIZE + 1);
            memcpy(message.data(), &HomeInfo::TeamShipBuffer[index], BUFFER_SIZE);
            api.SendBinaryMessage(index, message);
        }
    }
    void report(IShipAPI& api, unsigned char instruc, unsigned char param, coordinate target = {-1, -1})
    {
        ShipInfo::ReportBuffer = {instruc, param, target};
        std::string message;
        message.resize(REPORTBUFFER_SIZE + 1);
        memcpy(message.data(), &ShipInfo::ReportBuffer, REPORTBUFFER_SIZE);
        api.SendBinaryMessage(0, message);
    }
    std::deque<ReportBuffer> reports_to_send;
    //std::deque<int> report_ids;
    void set_report(ReportBuffer& report)
    {
        for (size_t i = 0; i < 4; i++)
        {
			HomeInfo::TeamShipBuffer[i].with_param = true;
			HomeInfo::TeamShipBuffer[i].instruction = report.instruction;
			HomeInfo::TeamShipBuffer[i].param = report.param;
			HomeInfo::TeamShipBuffer[i].param_pos = report.param_pos;
		}
    }
    void receive_message(ITeamAPI& api)
    {
        while (api.HaveMessage())
        {
            auto mes = api.GetMessage();
            std::string message = mes.second;
            Commute::ReportBuffer temp;
            memcpy(&temp, message.data(), REPORTBUFFER_SIZE);
            if (temp.instruction == Instruction_AttackState and temp.param == Parameter_AttackSuccess)
            {
                HomeInfo::TeamShipBuffer[mes.first].Mode = IDLE;
                continue;
            }
            else if (temp.instruction == Instruction_RefreshResource and temp.param == Parameter_ResourceRunningOut)
            {
                MapInfo::eraseResource(temp.param_pos);
                //if (MapInfo::PositionLists[MapInfo::Resource].empty())
                if (MapInfo::resource[1].empty())
                {
                    //HomeInfo::TeamShipBuffer[HomeInfo::index_to_id[0]].Mode = CONSTRUCT;
                    HomeInfo::TeamShipBuffer[HomeInfo::index_to_id[0]].ModeParam = 0;
                }
            }
            else if (temp.instruction == Instruction_RefreshConstruction)
            {
                if (temp.param == Parameter_ConstructionBuildUp)
                {
                    MapInfo::eraseConstruction(temp.param_pos);
                    if (MapInfo::construction[1].empty())
                    {
						HomeInfo::TeamShipBuffer[HomeInfo::index_to_id[0]].Mode = PRODUCE;
						HomeInfo::TeamShipBuffer[HomeInfo::index_to_id[0]].ModeParam = 1;
					}
                }
            }
            reports_to_send.push_back(temp);
        }
    }
    void process_message()
    {
        if (!reports_to_send.empty())
        {
            set_report(reports_to_send.front());
            reports_to_send.pop_front();
        }
    }
}





bool can_walk(int x, int y)
{
    MapInfo::Place t = MapInfo::fullmap[x][y];
    return (t == MapInfo::Space or t == MapInfo::Shadow or t == MapInfo::OpenWormhole);
}
bool can_attack(int x, int y)
{
    MapInfo::Place t = MapInfo::fullmap[x][y];
    return (t == MapInfo::Space or t == MapInfo::Shadow or t == MapInfo::OpenWormhole or t == MapInfo::ClosedWormhole);
}
bool can_see(int x, int y)
{
    MapInfo::Place t = MapInfo::fullmap[x][y];
    return t != MapInfo::Ruin;
}
class RoadSearch
{
public:
    coordinate target, last_location = {-1, -1};
    std::deque<coordinate> path;
    double last_angle = 0;
    bool visited[maxLen][maxLen];
    coordinate from[maxLen][maxLen];
    std::function<bool(IShipAPI& api)> end_condition;
    RoadSearch(coordinate target, std::function<bool(IShipAPI& api)> end_condition) :
        target(target),
        end_condition(std::move(end_condition))
    {
	}
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
            for (int i = -1; i <= 1; i++)
            {
                for (int j = -1; j <= 1; j++)
                {
                    if (i == 0 and j == 0)
                        continue;
                    if (i == -1 and j == -1 and (!can_walk(now.x - 1, now.y) or !can_walk(now.x, now.y - 1)))
                        continue;
                    if (i == 1 and j == -1 and (!can_walk(now.x + 1, now.y) or !can_walk(now.x, now.y - 1)))
                        continue;
                    if (i == -1 and j == 1 and (!can_walk(now.x - 1, now.y) or !can_walk(now.x, now.y + 1)))
                        continue;
                    if (i == 1 and j == 1 and (!can_walk(now.x + 1, now.y) or !can_walk(now.x, now.y + 1)))
                        continue;
                    int nx = now.x + i, ny = now.y + j;
                    if (nx < 0 or nx >= 50 or ny < 0 or ny >= 50 or visited[nx][ny])
                        continue;
                    if (nx == x2 and ny == y2)
                    {
                        if (i != 0 and j != 0)
                        {
                            continue;
                        }
                        double cost = sqrt(i * i + j * j);
                        pq.push({nx, ny, coordinate{now.x, now.y}, now.cost + cost, 0});
                    }
                    // if ((t != MapInfo::Space and t != MapInfo::Shadow and t != MapInfo::OpenWormhole) || (coordinate(nx, ny) == *MapInfo::NoStep.find(coordinate(nx, ny))))
                    if (!can_walk(nx, ny))
                        continue;
                    double cost = sqrt(i * i + j * j);
                    int h = manhatten_distance(nx, ny, x2, y2);
                    pq.push({nx, ny, coordinate{now.x, now.y}, now.cost + cost, (double)h});
                }
            }
        }
        return {};
    }
    bool operator()(IShipAPI &api)
    {
        std::cout << "RoadSearch TO:" << target.x << "," << target.y << "\n";
        if (target.x>49||target.x<0||target.y<0||target.y>49)
        {
            return true;
        }
        if (path.empty())
        {
            if (manhatten_distance(ShipInfo::myself.me.x / 1000, ShipInfo::myself.me.y / 1000, target) <= 1)
            {
                return true;
            }
            path = search_road(ShipInfo::myself.me.x / 1000, ShipInfo::myself.me.y / 1000, target.x, target.y);
        }
        if (end_condition(api))
        {
            return true;
        }
        if (api.GetSelfInfo()->shipState == THUAI7::ShipState::Idle and !path.empty())
        {
            std::vector<std::pair<coordinate, MapInfo::Place>> temp;
            auto saveAndChange = [&temp](int x, int y)
            {
                temp.push_back({{x, y}, MapInfo::fullmap[x][y]});
                MapInfo::fullmap[x][y] = MapInfo::Place::Ruin;
            };
            auto ProcessNearbyShip = [&saveAndChange](std::shared_ptr<const THUAI7::Ship> ship)
            {
                THUAI7::Ship& me = ShipInfo::myself.me;
                int x = ship->x / 1000, y = ship->y / 1000, lx = ship->x % 1000, ly = ship->y % 1000;
                int mx = ShipInfo::myself.me.x / 1000, my = ShipInfo::myself.me.y / 1000;
                if (euclidean_distance(ship->x, ship->y, me.x, me.y) < 2000)
                {
                    saveAndChange(x, y);
                    if (lx <= 420)
                    {
                        saveAndChange(x - 1, y);
                    }
                    else if (lx >= 580)
                    {
                        saveAndChange(x + 1, y);
                    }
                    if (ly <= 420)
                    {
                        saveAndChange(x, y - 1);
                    }
                    else if (ly >= 580)
                    {
                        saveAndChange(x, y + 1);
                    }
                }
            };
            for (auto const& ship : ShipInfo::FriendShips)
            {
                ProcessNearbyShip(ship);
            }
            for (auto const& ship : ShipInfo::Enemies)
            {
				ProcessNearbyShip(ship);
			}
            if (euclidean_distance(ShipInfo::myself.me.x, ShipInfo::myself.me.y, last_location.x, last_location.y) < 100)
            {
                last_location = {ShipInfo::myself.me.x, ShipInfo::myself.me.y};
                std::cout << "move backward" << std::endl;
                for (const auto& i : temp)
                {
                    MapInfo::fullmap[i.first.x][i.first.y] = i.second;
                }
                api.Move(300, last_angle + PI);
                return false;
			}
            else if (!temp.empty() or euclidean_distance(ShipInfo::myself.me.x, ShipInfo::myself.me.y, path.front().x * 1000 + 500, path.front().y * 1000 + 500) > 1500)
            {
                path = search_road(ShipInfo::myself.me.x / 1000, ShipInfo::myself.me.y / 1000, target.x, target.y);
                for (const auto& i : temp)
                {
                    MapInfo::fullmap[i.first.x][i.first.y] = i.second;
                }
                if (path.empty())
                {
                    return false;
                }
            }

            auto next = path.front();
            path.pop_front();
            int dx = next.x * 1000 + 500 - ShipInfo::myself.me.x;
            int dy = next.y * 1000 + 500 - ShipInfo::myself.me.y;
            double time = sqrt(dx * dx + dy * dy) / ShipInfo::myself.me.speed*1000;
            double angle = atan2(dy, dx);
            #ifdef DEBUG
            std::cout << "x: " << ShipInfo::myself.me.x << " y: " << ShipInfo::myself.me.y << "nx: " << next.x << "ny: " << next.y << std::endl;
            std::cout << "dx: " << dx << " dy: " << dy << std::endl;
            std::cout << "time: " << time << " angle: " << angle << std::endl;
            #endif
            last_angle = angle;
            last_location = {ShipInfo::myself.me.x, ShipInfo::myself.me.y};
            auto res = api.Move((int)time, angle);
            if (!res.get())
            {
                std::cout << "move fail"; 
            }
        }
        return false;
    }
};
class RoadSearchRange
{
public:
    coordinate &target, last_location = {-1, -1};
    std::deque<coordinate> path;
    double last_angle = 0;
    bool visited[maxLen][maxLen];
    const std::unordered_set<coordinate, PointHash>& des;
    coordinate from[maxLen][maxLen];
    std::function<bool(IShipAPI& api)> end_condition;
    RoadSearchRange(coordinate&tar, const std::unordered_set<coordinate, PointHash>& des, std::function<bool(IShipAPI& api)> end_condition) :
        target(tar),
            des(des),
            end_condition(std::move(end_condition))
    {
    }
    std::deque<coordinate> search_road(int x1, int y1, const std::unordered_set<coordinate, PointHash> &des, IShipAPI& api)
    {
        int min_dis = 0x7fffffff;
//        auto index = MapInfo::PlaceTypeConvert(type, 0, 0);
        for (const auto& i : des)
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
            for (int i = -1; i <= 1; i++)
            {
                for (int j = -1; j <= 1; j++)
                {
                    if (i == 0 and j == 0)
                        continue;
                    if (i == -1 and j == -1 and (!can_walk(now.x - 1,now.y) or !can_walk(now.x,now.y - 1)))
                        continue;
                    if (i == 1 and j == -1 and (!can_walk(now.x + 1,now.y) or !can_walk(now.x,now.y - 1)))
                        continue;
                    if (i == -1 and j == 1 and (!can_walk(now.x - 1,now.y) or !can_walk(now.x,now.y + 1)))
                        continue;
                    if (i == 1 and j == 1 and (!can_walk(now.x + 1,now.y) or !can_walk(now.x,now.y + 1)))
                        continue;
                    int nx = now.x + i, ny = now.y + j;
                    if (nx < 0 or nx >= 50 or ny < 0 or ny >= 50 or visited[nx][ny])
                        continue;
                    if (des.find({nx, ny}) != des.end() and (i == 0 or j == 0))
                    {
                        double cost = sqrt(i * i + j * j);
                        pq.push({nx, ny, coordinate{now.x, now.y}, now.cost + cost, 0});
                    }
                    THUAI7::PlaceType t = MapInfo::map[nx][ny];
                    //if (t != THUAI7::PlaceType::Space and t != THUAI7::PlaceType::Shadow and t != THUAI7::PlaceType::Wormhole)
                    if (!can_walk(nx, ny))
                        continue;
                    double cost = sqrt(i * i + j * j);
                    min_dis = 0x7fffffff;
                    for (auto const& k : des)
                    {
                        int temp = manhatten_distance(nx, ny, k);
                        if (temp < min_dis)
                            min_dis = temp;
                    }
                    pq.push({nx, ny, coordinate{now.x, now.y}, now.cost + cost, (double)min_dis});
                }
            }
        }
        return {};
    }
    bool operator()(IShipAPI &api)
    {
        std::cout << "RoadSearch TO:" << target.x << "," << target.y << "\n";
        /*
        if (target.x>49||target.x<0||target.y<0||target.y>49)
        {
            return true;
        }*/
        if (des.empty())
        {
            return true;
        }
        if (path.empty())
        {
            if (target.x == -1)
            {
                path = search_road(ShipInfo::myself.me.x / 1000, ShipInfo::myself.me.y / 1000, des, api);
                target = path.back();
            }
            else
            {
                if (manhatten_distance(ShipInfo::myself.me.x / 1000, ShipInfo::myself.me.y / 1000, target) <= 1)
                {
                    return true;
                }
                path = search_road(ShipInfo::myself.me.x / 1000, ShipInfo::myself.me.y / 1000, des, api);
                target = path.back();
            }
        }
        if (end_condition(api))
        {
            return true;
        }
        if (api.GetSelfInfo()->shipState == THUAI7::ShipState::Idle and !path.empty())
        {
            std::vector<std::pair<coordinate, MapInfo::Place>> temp;
            auto saveAndChange = [&temp](int x, int y)
            {
                temp.push_back({{x, y}, MapInfo::fullmap[x][y]});
                MapInfo::fullmap[x][y] = MapInfo::Place::Ruin;
            };
            auto ProcessNearbyShip = [&saveAndChange](std::shared_ptr<const THUAI7::Ship> ship)
            {
                THUAI7::Ship& me = ShipInfo::myself.me;
                int x = ship->x / 1000, y = ship->y / 1000, lx = ship->x % 1000, ly = ship->y % 1000;
                int mx = ShipInfo::myself.me.x / 1000, my = ShipInfo::myself.me.y / 1000;
                if (euclidean_distance(ship->x, ship->y, me.x, me.y) < 2000)
                {
                    saveAndChange(x, y);
                    if (lx <= 420)
                    {
                        saveAndChange(x - 1, y);
                    }
                    else if (lx >= 580)
                    {
                        saveAndChange(x + 1, y);
                    }
                    if (ly <= 420)
                    {
                        saveAndChange(x, y - 1);
                    }
                    else if (ly >= 580)
                    {
                        saveAndChange(x, y + 1);
                    }
                }
            };
            for (auto const& ship : ShipInfo::FriendShips)
            {
                ProcessNearbyShip(ship);
            }
            for (auto const& ship : ShipInfo::Enemies)
            {
                ProcessNearbyShip(ship);
            }
            if (euclidean_distance(ShipInfo::myself.me.x, ShipInfo::myself.me.y, last_location.x, last_location.y) < 100)
            {
                last_location = {ShipInfo::myself.me.x, ShipInfo::myself.me.y};
                std::cout << "move backward" << std::endl;
                for (const auto& i : temp)
                {
                    MapInfo::fullmap[i.first.x][i.first.y] = i.second;
                }
                api.Move(300, last_angle + PI);
                return false;
            }
            else if (!temp.empty() or euclidean_distance(ShipInfo::myself.me.x, ShipInfo::myself.me.y, path.front().x * 1000 + 500, path.front().y * 1000 + 500) > 1500)
            {
                path = search_road(ShipInfo::myself.me.x / 1000, ShipInfo::myself.me.y / 1000, des, api);
                target = path.back();
                for (const auto& i : temp)
                {
                    MapInfo::fullmap[i.first.x][i.first.y] = i.second;
                }
                if (path.empty())
                {
                    return false;
                }
            }

            auto next = path.front();
            path.pop_front();
            int dx = next.x * 1000 + 500 - ShipInfo::myself.me.x;
            int dy = next.y * 1000 + 500 - ShipInfo::myself.me.y;
            double time = sqrt(dx * dx + dy * dy) / ShipInfo::myself.me.speed*1000;
            double angle = atan2(dy, dx);
#ifdef DEBUG
            std::cout << "x: " << ShipInfo::myself.me.x << " y: " << ShipInfo::myself.me.y << "nx: " << next.x << "ny: " << next.y << std::endl;
            std::cout << "dx: " << dx << " dy: " << dy << std::endl;
            std::cout << "time: " << time << " angle: " << angle << std::endl;
#endif
            last_angle = angle;
            last_location = {ShipInfo::myself.me.x, ShipInfo::myself.me.y};
            auto res = api.Move((int)time, angle);
            if (!res.get())
            {
                std::cout << "move fail";
            }
        }
        return false;
    }
};




struct callFunc
{
    std::function<bool(IShipAPI&)> func;
    int id, priority;
    bool operator<(const callFunc& a) const
    {
		return priority < a.priority;
	}
};

//std::stack<std::function<bool(IShipAPI&)>> callStack;
std::unordered_set<int> interrupt_codeRecorder;
std::priority_queue<callFunc> callStack;


inline bool Myside(int i)
{
    return (i==ShipInfo::myself.me.teamID);
}
inline bool ConstructionFullHp(int curHp, THUAI7::ConstructionType a)
{
    return (curHp >= ((a == THUAI7::ConstructionType::Factory) ? 12000 : ((a == THUAI7::ConstructionType::Fort) ? 16000 : 10000)));
}

namespace IdleMode
{
    std::unordered_set<coordinate, PointHash> InspectionList;
    coordinate target = {-1, -1};

    inline bool LoadNearestPosition(IShipAPI& api)
    {
        coordinate tmp;
        int x = ShipInfo::myself.me.x / 1000;  //<格子数
        int y = ShipInfo::myself.me.y / 1000;  //<格子数
        int minDis = 0x7fffffff;
        if (InspectionList.empty())
        {
            return false;
        }
        for (auto const& c : InspectionList)
        {
            if (manhatten_distance(x, y, c) < minDis)
            {
                tmp = c, minDis = manhatten_distance(x, y, c);
            }
        }
        target = tmp;
        //InspectionList.erase(tmp);
        return true;
    }

    auto near_enough = [](IShipAPI& api)
    {
        // return (manhatten_distance(ShipInfo::myself.me.x / 1000, ShipInfo::myself.me.y / 1000, ShipInfo::myself.TargetPos.x, ShipInfo::myself.TargetPos.y) <= 1);
        return (euclidean_distance(ShipInfo::myself.me.x / 1000, ShipInfo::myself.me.y / 1000, target.x, target.y) <= 1);
    };

    bool Perform(IShipAPI& api)
    {
        if (ShipInfo::myself.me.playerID>=3)
        {
            if (near_enough(api))
            {
                auto construction_stat = api.GetConstructionState(target.x, target.y);
                if(!construction_stat.has_value())
                {
                    InspectionList.erase(target);
                    target = {-1, -1};
                    return true;
                }
                auto construction = construction_stat.value();
                if(Myside(construction.teamID))
                {
                    InspectionList.erase(target);
                    target = {-1, -1};
                    if(!ConstructionFullHp(construction.hp, construction.constructionType)){
                        Commute::report(api, Instruction_RefreshConstruction, Parameter_DestroyedFriendConstruction, target);
                    }
                    return true;
                }
                if (construction.hp > 1000)
                {
                    if (ShipInfo::myself.me.shipState != THUAI7::ShipState::Attacking and ShipInfo::myself.me.shipState != THUAI7::ShipState::Swinging)
                    {
                        api.Attack(atan2(target.y * 1000 + 500 - ShipInfo::myself.me.y, target.x * 1000 + 500 - ShipInfo::myself.me.x));
                        #ifdef DEBUG
                        cout << "dy: " << target.y * 1000 + 500 - ShipInfo::myself.me.y;
                        cout << " dx: " << target.x * 1000 + 500 - ShipInfo::myself.me.x << endl;
                        cout << "attack angle: ";
                        #endif
                        std::cout << atan2(target.y * 1000 + 500 - ShipInfo::myself.me.y, target.x * 1000 + 500 - ShipInfo::myself.me.x) << std::endl;
                    }
                    return false;
                }
                else
                {
                    InspectionList.erase(target);
                    api.EndAllAction();
                    target = {-1, -1};
                    Commute::report(api, Instruction_RefreshConstruction, Parameter_DestroyedEnemyConstruction, target);
                    return true;
                }
            }
            else
            {
                if (InspectionList.empty())
                {
                    //InspectionList = MapInfo::Ori_PositionLists[MapInfo::Place::Construction];
                    for (auto const& i : MapInfo::PositionLists[MapInfo::Construction])
                    {
						InspectionList.insert(i);
					}
                }
                LoadNearestPosition(api);
                std::cout << "Triggered RoadSearch,target:" << target.x << "," << target.y << "\n";
                auto search = std::make_shared<RoadSearch>(target, near_enough);
                int priority = PRIORITY_Normal * RATIO + callStack.size();
                callStack.push({*search, RoadSearchID, priority});
                return false;
            }
        }
        return false;
    }

    void Clear(IShipAPI& api)
    {
    }
}

namespace ConstructMode
{
    coordinate target = {-1, -1};
    bool side = true;

    /*
    bool GetNearestConstruction(IShipAPI& api)
    {
        coordinate tmp;
        int x = ShipInfo::myself.me.x / 1000;  //<格子数
        int y = ShipInfo::myself.me.y / 1000;  //<格子数
        int minDis = 0x7fffffff;
        if (MapInfo::PositionLists[MapInfo::Construction].empty())
        {
			return false;
		}
        for (auto const& c : MapInfo::PositionLists[MapInfo::Construction])
        {
            if (manhatten_distance(x, y, c) < minDis)
            {
				tmp = c, minDis = manhatten_distance(x, y, c);
			}
		}
        target = tmp;
        return true;
    }*/

    auto near_enough = [](IShipAPI& api)
    {
        // return (manhatten_distance(ShipInfo::myself.me.x / 1000, ShipInfo::myself.me.y / 1000, ShipInfo::myself.TargetPos.x, ShipInfo::myself.TargetPos.y) <= 1);
        return (euclidean_distance(ShipInfo::myself.me.x / 1000, ShipInfo::myself.me.y / 1000, target.x, target.y) <= 1);
    };

    bool Perform(IShipAPI& api)
    {
        if (ShipInfo::myself.me.shipState == THUAI7::ShipState::Constructing)
        {
            return false;
        }
        if (MapInfo::PositionLists[MapInfo::Construction].empty())
        {
			return false;
		}
        if (near_enough(api))
        {
            std::cout << "near enough" << std::endl;
            auto res = api.GetConstructionState(target.x, target.y);
            if(!res.has_value()){
                std::cout << "no construction" << std::endl;
                if (ShipInfo::myself.me.shipState != THUAI7::ShipState::Constructing)
                {
                    api.EndAllAction();
                    api.Construct(ShipInfo::constructType);
                }
                return false;
            }
            auto construction = res.value();
            if (!Myside(construction.teamID))
            {
                std::cout << "enemy construction" << std::endl;
                MapInfo::eraseConstruction(target);
                //TODO! 通知军舰攻击建筑
                Commute::report(api, Instruction_RefreshConstruction, Parameter_EnemyBuildConstruction, target);
                target = {-1, -1};
                return true;
            }
            else if (ConstructionFullHp(construction.hp, construction.constructionType))
            {
                std::cout << "full hp construction" << std::endl;
                MapInfo::eraseConstruction(target);
                Commute::report(api, Instruction_RefreshConstruction, Parameter_ConstructionBuildUp, target);
                target = {-1, -1};
                return true;
            }
            else
            {
                std::cout << "half hp construction" << std::endl;
                if (ShipInfo::myself.me.shipState != THUAI7::ShipState::Constructing)
                {
                    api.EndAllAction();
                    api.Construct(construction.constructionType);
                }
            }
        }
        else
        {
            //if (!GetNearestConstruction(api))
            //    return false;
            std::cout << "Triggered RoadSearch,target:" << target.x << "," << target.y << "\n";
            auto search = std::make_shared<RoadSearchRange>(target, MapInfo::construction[side], near_enough);
            int priority = PRIORITY_Normal * RATIO + callStack.size();
            callStack.push({*search, RoadSearchID, priority});
        }
        return false;
    }
}

namespace ProduceMode
{

    coordinate target = {-1, -1};
    bool side = true;
    /*
    bool GetNearestResource(IShipAPI& api)
    {
        coordinate tmp;
        int x = ShipInfo::myself.me.x / 1000;  //<格子数
        int y = ShipInfo::myself.me.y / 1000;  //<格子数
        int minDis = 0x7fffffff;
        if (MapInfo::PositionLists[MapInfo::Resource].empty())
        {
			return false;
		}
        for (auto const& c : MapInfo::PositionLists[MapInfo::Resource])
        {
            if (manhatten_distance(x, y, c) < minDis)
            {
				tmp = c, minDis = manhatten_distance(x, y, c);
			}
		}
        target = tmp;
        return true;
    }*/
    auto near_enough = [](IShipAPI& api)
    {
        return (euclidean_distance(ShipInfo::myself.me.x / 1000, ShipInfo::myself.me.y / 1000, target.x, target.y) <= 1);
    };

    bool Perform(IShipAPI& api)
    {
        if (ShipInfo::myself.me.shipState == THUAI7::ShipState::Producing)
        {
			return false;
		}
        if (near_enough(api))
        {
            if (api.GetResourceState(target.x, target.y)>0)
            {
                if (ShipInfo::myself.me.shipState != THUAI7::ShipState::Producing)
                {
                    api.EndAllAction();
                    api.Produce();
                }
            }
            else
            {
                MapInfo::eraseResource(target);
                Commute::report(api, Instruction_RefreshResource, Parameter_ResourceRunningOut, target);
                target = {-1, -1};
                return true;
            }
        }
        else
        {
            /*
            if (!GetNearestResource(api))
            {
                return false;
            }*/
            std::cout << "Triggered RoadSearch,target:" << target.x << "," << target.y << "\n";

            auto search = std::make_shared<RoadSearchRange>(target, MapInfo::resource[side], near_enough);
            int priority = PRIORITY_Normal * RATIO + callStack.size();
            callStack.push({*search, RoadSearchID, priority});

        }
        return false;
    }

    void Clear(IShipAPI& api)
    {

    }
}

namespace AttackMode
{
    std::queue<coordinate> target;

    //返回true表示这个地方的东西已经消灭了 所以应该是没血才返回true
    


    /**
     * @brief 检查目标是否被摧毁
     * @param api 
     * @param tar 目标所在格子数
     * @return true: 已成功消灭；false: 未消灭
     */
    bool checkState(IShipAPI& api, coordinate tar){
        THUAI7::PlaceType temp = MapInfo::map[tar.x][tar.y];
        if(temp == THUAI7::PlaceType::Construction){
            auto res = api.GetConstructionState(tar.x, tar.y);
            if(!res.has_value()){
                return true;
            }else{
                auto construction = res.value();
                if(Myside(construction.teamID)){
                    return true;
                }else if(construction.hp <= 0) {
                    Commute::report(api, Instruction_RefreshConstruction, Parameter_DestroyedEnemyConstruction, tar);
                    return true;
                }else{
                    return false;
                }
            }
        }
        else if(temp == THUAI7::PlaceType::Home){
            return (MapInfo::MySide == RED) ? (api.GetGameInfo()->blueHomeHp <= 0) : (api.GetGameInfo()->redHomeHp <= 0);
            //TODO! 基地得知对方基地被消灭之后要做什么
        }
        else if(temp == THUAI7::PlaceType::Wormhole){
            return api.GetWormholeHp(tar.x, tar.y) <= 0;
            //TODO! 虫洞被消灭之后要做什么
        }
        else{
            return false;
        }
    }


    bool Perform(IShipAPI& api)
    {
        if (!target.empty())
        {
            auto cur_target = target.front();
            if (euclidean_distance(ShipInfo::myself.me.x, ShipInfo::myself.me.y, cur_target.x * 1000 + 500, cur_target.y * 1000 + 500)
                <= WeaponToDis(ShipInfo::myself.me.weaponType) - 200)
            {
                if (checkState(api, cur_target))
                {
                    target.pop();
                    if (target.empty())
                    {
                        Commute::report(api, Instruction_AttackState, Parameter_AttackSuccess);
                    }
                    return false;
                }
                if (ShipInfo::myself.me.shipState != THUAI7::ShipState::Idle)
                {
                    api.EndAllAction();
                }

                double angle = atan2(cur_target.y * 1000 + 500 - ShipInfo::myself.me.y, cur_target.x * 1000 + 500 - ShipInfo::myself.me.x);
                // double angle = -0.3;
                std::cout << "mydirect: " << ShipInfo::myself.me.facingDirection << std::endl;
                std::cout << "angle: " << angle << std::endl;
                //api.Attack(angle);
                coordinate tmp = MyAttack(api, angle, target.front());
                if (tmp.x < 0)
                {
                    std::cout << "NOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO!!!!\n";
                }
                else
                {
                    auto end_condition = [](IShipAPI& api)
                    {
                        return false;
                    };
                    std::cout << "Triggered RoadSearch,target:" << tmp.x << "," << tmp.y << "\n";

                    auto search = std::make_shared<RoadSearch>(tmp, end_condition);
                    int priority = PRIORITY_Normal * RATIO + callStack.size();
                    callStack.push({*search, RoadSearchID, priority});
                }
            }
            else
            {
                auto end_condition = [cur_target](IShipAPI& api)
                {
                    return (euclidean_distance(ShipInfo::myself.me.x, ShipInfo::myself.me.y, cur_target.x * 1000 + 500, cur_target.y * 1000 + 500)
                        <= WeaponToDis(ShipInfo::myself.me.weaponType) and api.HaveView(cur_target.x * 1000 + 500, cur_target.y * 1000 + 500));
                };
                std::cout << "Triggered RoadSearch,target:" << cur_target.x << "," << cur_target.y << "\n";

                auto search = std::make_shared<RoadSearch>(cur_target, end_condition);
                int priority = PRIORITY_Normal * RATIO + callStack.size();
                callStack.push({*search, RoadSearchID, priority});
            }
        }
        return false;
    }
}


//ModeRetval (*perform_list[3])(IShipAPI&) = {&(IdleMode::Perform), &(RoadSearchMode::Perform), &(ProduceMode::Perform)};
//void (*clear_list[3])(IShipAPI&) = {&(IdleMode::Clear), &(RoadSearchMode::Clear), &(ProduceMode::Clear)};


ShipMode nextMode=IDLE;

bool ShipStep(IShipAPI &api);



/**
 * @brief 检测我方是否在对方射程内
 * @param x 敌方x坐标
 * @param y 敌方y坐标
 * @param weapondistance 武器射程 
 * @return 
 */
inline bool In_ShootingDistance(int x, int y, int weapondistance)
{
    if (euclidean_distance(ShipInfo::myself.me.x, ShipInfo::myself.me.y,x,y)<=weapondistance+400+800)
    {
        return true;
    }
    else
    {
        return false;
    }
}




double WeaponHarm[6][3] = {
    {800, 1.5, 0.6},
    {1000, 2, 0.4},
    {1200, 0.4, 1.5},
    {1100, 1, 1000},
    {1600, 2, 2}
};
struct health
{
    int hp;
    int armor;
    int shield;

    void operator-=(THUAI7::WeaponType a)
    {
        int index = (int)a;
        double damage = WeaponHarm[index][0];
        double damage_to_shield = damage * WeaponHarm[index][2];
        if (shield > 0 and a != THUAI7::WeaponType::MissileGun)
        {
			shield = (damage_to_shield > shield) ? 0 : (shield - damage_to_shield);
		}
        else if (armor > 0)
        {
            double damage_to_armor = damage * WeaponHarm[index][1];
			armor = (damage_to_armor > armor) ? 0 : (armor - damage_to_armor);
		}
        else if (hp > 0)
        {
			hp = (damage > hp) ? 0 : (hp - damage);
        }
        
    }
};

/**
 * @brief 判断能否打过
 * @param enemy 对应的敌舰
 * @return 0：打不过；1：打得过；2：躲草里能打过
 */
inline unsigned char Conquerable(THUAI7::Ship & enemy)
{
    coordinate me = {ShipInfo::myself.me.x, ShipInfo::myself.me.y};
    coordinate en = {enemy.x,enemy.y};

    health myhealth = {ShipInfo::myself.me.hp, ShipInfo::myself.me.armor, ShipInfo::myself.me.shield};
    health enemyhealth = {enemy.hp,enemy.armor,enemy.shield};

    THUAI7::WeaponType myweapon = ShipInfo::myself.me.weaponType;
    THUAI7::WeaponType enemyweapon = enemy.weaponType;

    if (MapInfo::fullmap[me.x / 1000][me.y / 1000] == MapInfo::Shadow && MapInfo::fullmap[en.x / 1000][en.y / 1000] != MapInfo::Shadow)
    {
        enemyhealth -= myweapon;
        while ((myhealth.hp && enemyhealth.hp))
        {
            enemyhealth -= myweapon;
            if (!enemyhealth.hp)
            {
                break;
            }
            myhealth -= enemyweapon;
            if (!myhealth.hp)
            {
                break;
            }
        }
        switch (myhealth.hp)
        {
            case 0:
                return 0;
                break;
            default:
                return 1;
                break;
        }
    }
    else
    {
        while ((myhealth.hp && enemyhealth.hp))
        {
            enemyhealth -= myweapon;
            if (!enemyhealth.hp)
            {
                break;
            }
            myhealth -= enemyweapon;
            if (!myhealth.hp)
            {
                break;
            }
        }
        switch (myhealth.hp)
        {
            case 0:
                enemyhealth -= myweapon;
                if (!enemyhealth.hp)
                {
                    return 2;
                }
                else
                {
                    return 0;
                }
                break;
            default:
                return 1;
                break;
        }
    }




}

auto MyRecovery = [] (IShipAPI & api)
{
    std::cout << "Enter RecoveryMode\n";
    int fullHp = ((ShipInfo::myself.me.shipType == THUAI7::ShipType::CivilianShip) ? (3000) : ((ShipInfo::myself.me.shipType == THUAI7::ShipType::MilitaryShip) ? 4000 : 12000));
    if (ShipInfo::myself.me.hp >= fullHp-1)
    {
        
        std::cout << "RecoveryMode：Full\n";
        return true;  // 血量回满则退出
    }
    else
    {
        if (ShipInfo::myself.me.shipState==THUAI7::ShipState::Recovering)
        {
            std::cout << "RecoveryMode：Continue\n";

            return false;//正在回血则继续
        }
        else
        {
            if (manhatten_distance(ShipInfo::myself.me.x / 1000, ShipInfo::myself.me.y / 1000, *MapInfo::PositionLists[MapInfo::MyHome].begin()) <= 1)
            {
                auto reply = api.Recover((api.GetEnergy() > fullHp - ShipInfo::myself.me.hp) ? fullHp - ShipInfo::myself.me.hp : api.GetEnergy());
                std::cout << "Recover Called, status:" << (reply.get() ? "success\n" : "fail\n");

                return false;
            }
            else
            {
                std::cout << "RecoveryMode：RoadSearch\n";

                //没到家就寻路
                static coordinate recoverypoint = (MapInfo::MySide == RED) ? coordinate{2, 47} : coordinate{47,2};
                auto search = std::make_shared<RoadSearch>(recoverypoint, [](IShipAPI& api)
                                                           { return (euclidean_distance(ShipInfo::myself.me.x / 1000, ShipInfo::myself.me.y / 1000, (*MapInfo::PositionLists[MapInfo::MyHome].begin()).x, (*MapInfo::PositionLists[MapInfo::MyHome].begin()).y) <= 1); });
                int priority = PRIORITY_Recovery * RATIO + callStack.size();
                callStack.push({*search, RoadSearchID, priority});

                return false;
            }
        }
    }
};

/**
 * @brief 巡视我方；
 * 优先级：0.5
 */
auto Inspecting = [](IShipAPI& api) 
{
    static int initialized = 0;
    static int x_position;
    static int y_position[3] = {11, 25, 39};
    static int cur_hole = 1;
    static int worm_x;
    //    static int dir = 1;
    if (!initialized)
    {
        if (MapInfo::MySide == RED)
        {
//            dir = -1;
//            x_position = 1000;
            x_position = 21;
            worm_x = 23;
        }
        else
        {
            x_position = 28;
            worm_x = 26;
        }
        /*
        for (auto const& i : MapInfo::PositionLists[MapInfo::OpenWormhole])
        {
            if ((i.x + dir - x_position) * dir>0)
            {
                x_position = i.x + dir;
            }
            if (i.y < 16 && i.y > y_position[0])
            {
                y_position[0] = i.y;
            }
            else if (i.y > 30 && i.y < y_position[2])
            {
                y_position[2] = i.y;
            }
            else if (i.y>16&&i.y<30)
            {
                y_position[1] = i.y;
            }
        }
        for (auto const& i : MapInfo::PositionLists[MapInfo::ClosedWormhole])
        {
            if ((i.x + dir - x_position) * dir > 0)
            {
                x_position = i.x + dir;
            }
            if (i.y < 16 && i.y > y_position[0])
            {
                y_position[0] = i.y;
            }
            else if (i.y > 30 && i.y < y_position[2])
            {
                y_position[2] = i.y;
            }
            else if (i.y > 16 && i.y < 30)
            {
                y_position[1] = i.y;
            }
        }*/
        initialized = 1;
    }
    int distance = manhatten_distance(ShipInfo::myself.me.x / 1000, ShipInfo::myself.me.y / 1000, x_position, y_position[cur_hole]);
    if (distance <= 1)
    {
        std::cout << "Inspecting:: 1\n";
        /*
        if (distance>0)
        {
            std::cout << "Inspecting:: 2\n";
            if (ShipInfo::myself.me.shipState != THUAI7::ShipState::Moving)
            {
                std::cout << "Inspecting:: 3\n";
                api.EndAllAction();
                double angle = atan2(y_position[cur_hole] * 1000 + 500 - ShipInfo::myself.me.y, x_position * 1000 + 500 - ShipInfo::myself.me.x);
                double time = euclidean_distance(x_position * 1000 + 500, y_position[cur_hole] * 1000 + 500, ShipInfo::myself.me.x, ShipInfo::myself.me.y) / ShipInfo::myself.me.speed * 1000;
                api.Move((int)time, angle);
            }

            return false;
        }*/
        std::cout << "Inspecting:: 4\n";

        int holehp = api.GetWormholeHp(worm_x, y_position[cur_hole]);
        std::cout << "Inspecting:: 5\n";
        // 如果原本记录为关闭，则记录为打开，并报告
        coordinate tmp(worm_x, y_position[cur_hole]);
        if (holehp >= 12000 && MapInfo::fullmap[tmp.x][tmp.y] == MapInfo::ClosedWormhole)
        {
            openWormhole(tmp);
            Commute::report(api, WormholeOpen, 0, tmp);
        }
        //如果允许攻击，则攻击
        if (holehp>0&&ShipInfo::WhetherAttackWormhole)
        {
            std::cout << "Inspecting:: 6\n";
            coordinate result = MyAttack(api, atan2(tmp.y * 1000 + 500 - ShipInfo::myself.me.y, tmp.x * 1000 + 500 - ShipInfo::myself.me.x), tmp);
        }

        if (holehp<12000 and MapInfo::fullmap[tmp.x][tmp.y] == MapInfo::OpenWormhole)
        {
            closeWormhole(tmp);
            Commute::report(api, WormholeDestroyed, 0, tmp);
        }

        if (holehp <= 100)
        {
            std::cout << "Inspecting:: 7\n";
            cur_hole = (cur_hole + 1) % 3;
        }
        return false;

    }
    else//没到地点，寻路
    {
        std::cout << "Inspecting:: 8\n";
        coordinate tmp = {x_position, y_position[cur_hole]};
        std::cout << "Triggered RoadSearch,target:" << tmp.x << "," << tmp.y << "\n";

        auto search = std::make_shared<RoadSearch>(tmp, [](IShipAPI& api)
                                                   { return false;});
        int priority = PRIORITY_Inspection * RATIO + callStack.size();
        callStack.push({*search, RoadSearchID, priority});
        return false;
    }

};


/**
 * @brief 对应的Interruptcode:1
 */
/*
auto detectEnemy=[](IShipAPI& api)
{ 
        std::cout << "Detected Enemies!!!\n";
       
    if (!ShipInfo::Enemies.empty())
    {
        callStack.pop();
        interrupt_codeRecorder.pop();
        return;
    }
    for (auto const& i : ShipInfo::Enemies)
    {
        if (i->shipState == THUAI7::ShipState::Attacking && In_ShootingDistance(i->x, i->y, WeaponToDis(i->weaponType)))
        {
            api.EndAllAction();
            double angle = i->facingDirection;
            double move_angle = angle + PI / 2;
            auto dodgeNode = new BT::SequenceNode<IShipAPI>{
                new BT::eventNode<IShipAPI>{Conditions::always_ship, ShipAction::MoveFunc(move_angle, 200)},
                new BT::eventNode<IShipAPI>{Conditions::JudgeSelfState(THUAI7::ShipState::Idle), ShipAction::AttackFunc(i->x, i->y)},
            };

            callStack.push(WrapperFunc([dodgeNode](IShipAPI& api)
                                        { return dodgeNode->perform(api); }));
            break;
        }
    } 
};*/

inline coordinate judge_attack_pos(IShipAPI& api,double angle0,coordinate target1)
{
    int fulldistance = WeaponToDis(ShipInfo::myself.me.weaponType);
    double distotar = euclidean_distance(ShipInfo::myself.me.x, ShipInfo::myself.me.y, target1.x, target1.y);

    int real_x = target1.x + distotar * cos(angle0);
    int real_y = target1.y + distotar * sin(angle0);

    if (euclidean_distance(ShipInfo::myself.me.x, ShipInfo::myself.me.y, real_x, real_y) >= 650 && distotar <= fulldistance + 600)
    {
        return {
            -1, -1
        };
    }
    fulldistance = (fulldistance > distotar) ? distotar : fulldistance;
    double realangle = atan2(ShipInfo::myself.me.y - target1.y, ShipInfo::myself.me.x - target1.x);



    coordinate target(target1.x / 1000, target1.y / 1000);

    bool ok = true;
    static int r = 200;
    double angle;
    bool position_OK = false;
    coordinate position = {-1, -1};
    for (size_t angle_count = 1; angle_count < 5 &&!position_OK; angle_count++)
    {
        angle = angle0 + ((angle_count % 2 == 0 ? 1 : -1)) * (angle_count / 2 + angle_count % 2) * asin((double)650/fulldistance);
        int x = target1.x + fulldistance * cos(angle);
        int y = target1.y + fulldistance * sin(angle);
        for (double i = 400; i < fulldistance and ok; i += 400)
        {
            int x1 = x - cos(angle) * i;
            int y1 = y - sin(angle) * i;
            int x0 = (x1 / 1000) * 1000;
            int y0 = (y1 / 1000) * 1000;
            int lx = x1 - x0;
            int ly = y1 - y0;
            std::vector<coordinate> judge;

            coordinate tmp(x1 / 1000, y1 / 1000);  // 格子数的坐标
            THUAI7::PlaceType tmptype = MapInfo::map[tmp.x][tmp.y];
            if (tmp == target)
            {
                break;
            }
            if (tmp != target and !can_attack(tmp.x, tmp.y))
            //(tmptype == THUAI7::PlaceType::Asteroid || tmptype == THUAI7::PlaceType::Construction || tmptype == THUAI7::PlaceType::Home || tmptype == THUAI7::PlaceType::Resource || tmptype == THUAI7::PlaceType::Ruin || (MapInfo::PositionLists[MapInfo::ClosedWormhole].find(tmp) != MapInfo::PositionLists[MapInfo::ClosedWormhole].end())))
            {
                break;
            }
            if (lx < r)
            {
                judge.push_back({tmp.x - 1, tmp.y});
            }
            else if (lx > 1000 - r)
            {
                judge.push_back({tmp.x + 1, tmp.y});
            }
            if (ly < r)
            {
                judge.push_back({tmp.x, tmp.y - 1});
            }
            else if (ly > 1000 - r)
            {
                judge.push_back({tmp.x, tmp.y + 1});
            }
            if (euclidean_distance(x1, y1, x0, y0) < r)
            {
                judge.push_back({tmp.x - 1, tmp.y - 1});
            }
            else if (euclidean_distance(x1, y1, x0 + 1000, y0) < r)
            {
                judge.push_back({tmp.x + 1, tmp.y - 1});
            }
            else if (euclidean_distance(x1, y1, x0 + 1000, y0 + 1000) < r)
            {
                judge.push_back({tmp.x + 1, tmp.y + 1});
            }
            else if (euclidean_distance(x1, y1, x0, y0 + 1000) < r)
            {
                judge.push_back({tmp.x - 1, tmp.y + 1});
            }
            for (auto& i : judge)
            {
                if (i.x < 0 or i.x >= 50 or i.y < 0 or i.y >= 50)
                {
                    continue;
                }
                if (i != target and !can_attack(i.x, i.y))
                {
                    break;
                }
            }
            position_OK = true;
        }
        if (position_OK)
        {
            position = {x, y};
        }
    }
    if (euclidean_distance(position.x,position.y,ShipInfo::myself.me.x,ShipInfo::myself.me.y)<=100)
    {
        return {
            -1, -1
        };
    }
    else if (position.x==-1)
    {
        return {
            -2, -2
        };
    }
    else
    {
        return position;
    }

   
}

int Ship_Init = 1;

int Confrontation_Time_Count = -1;

void AI::play(IShipAPI& api)
{
    ShipInfo::CheckInfo(api);
    if (Ship_Init)
    {
        //if (api.GetSelfInfo()->playerID>2)
        //{
        //    nextMode = ATTACK;
        //    coordinate tmmmmp(26, 9);
        //    AttackMode::target.push(tmmmmp);
        //}
        //else
        //{
        //    nextMode = PRODUCE;
        //}
        nextMode = IDLE;
        callStack.push({&ShipStep, ShipStepID, 0});
        interrupt_codeRecorder.insert(0);
        Ship_Init = 0;
    }
    if (Commute::RefreshInfo(api))
    {
        if (ShipInfo::myself.mode!=ShipInfo::ShipBuffer.Mode)
        {
            //clear(api);
            nextMode = ShipInfo::myself.mode = ShipInfo::ShipBuffer.Mode;
            switch (ShipInfo::ShipBuffer.Mode)
            {
                case PRODUCE:
                    ProduceMode::side = ShipInfo::ShipBuffer.ModeParam;
                    if (ShipInfo::ShipBuffer.with_target)
                    {
                        ProduceMode::target = ShipInfo::ShipBuffer.target;
                    }
                    break;
                case CONSTRUCT:
                    ConstructMode::side = ShipInfo::ShipBuffer.ModeParam >= 0;
                    if (ShipInfo::ShipBuffer.with_target)
                    {
                        ConstructMode::target = ShipInfo::ShipBuffer.target;
                    }
                    if (ShipInfo::ShipBuffer.ModeParam < 0)
                    {
                        ConstructMode::side = 0;
                        ShipInfo::ShipBuffer.ModeParam = -ShipInfo::ShipBuffer.ModeParam;
                    }
                    switch (ShipInfo::ShipBuffer.ModeParam)
                    {
                        case MODEPARAM_ConstructFactory:
                            ShipInfo::constructType = THUAI7::ConstructionType::Factory;
                            break;
                        case MODEPARAM_ConstructFort:
                            ShipInfo::constructType = THUAI7::ConstructionType::Fort;
                            break;
                        case MODEPARAM_ConstructCommunity:
                            ShipInfo::constructType = THUAI7::ConstructionType::Community;
                            break;
                        default:
                            break;
                    }
                    break;
                case ATTACK:
                    switch (ShipInfo::ShipBuffer.ModeParam)
                    {
                        case MODEPARAM_AttackHome:
                            AttackMode::target.push(*(MapInfo::PositionLists[MapInfo::EnemyHome].begin()));
                            break;
                        default:
                            AttackMode::target.push(ShipInfo::ShipBuffer.target);
                            break;
                    }
                    break;
                case INSPECT:
                    break;
                default:
                    break;
            }
        }
        cout << "Mode Changed to " << nextMode << endl;
        if (ShipInfo::ShipBuffer.with_param)
        {
            switch (ShipInfo::ShipBuffer.instruction)
            {
                case Instruction_RefreshResource:
                    switch (ShipInfo::ShipBuffer.param)
                    {
                        case Parameter_ResourceRunningOut:
                            MapInfo::eraseResource(ShipInfo::ShipBuffer.param_pos);
                            break;
                        default:
                            break;
                    }
                    break;
                case Instruction_RefreshConstruction:
                    switch (ShipInfo::ShipBuffer.param)
                    {
                        case Parameter_ConstructionBuildUp:
                            MapInfo::eraseConstruction(ShipInfo::ShipBuffer.param_pos);
                            break;
                        case Parameter_EnemyBuildConstruction:
                            if (ShipInfo::myself.me.playerID >= 3)
                            {
                                AttackMode::target.push(ShipInfo::ShipBuffer.param_pos);
                            }
                            break;
                        case Parameter_DestroyedEnemyConstruction:
                            MapInfo::insertConstruction(ShipInfo::ShipBuffer.param_pos);
                            break;
                        case Parameter_DestroyedFriendConstruction:
                            MapInfo::insertConstruction(ShipInfo::ShipBuffer.param_pos);
                            break;
                        default:
                            break;
                    }
                    break;
                case WormholeDestroyed:
                    closeWormhole(ShipInfo::ShipBuffer.param_pos);
                    break;
                case WormholeOpen:
                    openWormhole(ShipInfo::ShipBuffer.param_pos);
                    break;
                default:
                    break;
            }
        }
    }

    
    /**
    * @brief 血量过低则回城
    */
    if (ShipInfo::myself.me.hp < 2000)
    {
        if (interrupt_codeRecorder.find(RecoveryID)==interrupt_codeRecorder.end())
        {
            interrupt_codeRecorder.insert(RecoveryID);
            int pri = PRIORITY_Recovery * RATIO + callStack.size();
            callStack.push({MyRecovery, RecoveryID, pri});
        }
    }


    for (auto const& i : ShipInfo::Enemies)
    {
        double dis = euclidean_distance(i->x, i->y, ShipInfo::myself.me.x, ShipInfo::myself.me.y);
        if (dis >= 8000)
        {
            continue;
        }
        if (i->weaponType == THUAI7::WeaponType::NullWeaponType)
        {
            if (ShipInfo::myself.me.weaponType != THUAI7::WeaponType::NullWeaponType)
            {
                auto& state = ShipInfo::myself.me.shipState;
                if (state != THUAI7::ShipState::Attacking and state != THUAI7::ShipState::Swinging)
                {
                    double dis = euclidean_distance(ShipInfo::myself.me.x, ShipInfo::myself.me.y, i->x, i->y);
                    if (dis < WeaponToDis(ShipInfo::myself.me.weaponType))
                    {
                        api.EndAllAction();
                        api.Attack(atan2(i->y - ShipInfo::myself.me.y, i->x - ShipInfo::myself.me.x));
                    }
                }
            }
            else
            {
                //TODO! 遇到无武装敌舰报告基地
            }
        }
        else
        {
            if (ShipInfo::myself.me.weaponType == THUAI7::WeaponType::NullWeaponType )
            {
                if ((euclidean_distance(ShipInfo::myself.me.x, ShipInfo::myself.me.y, i->x, i->y) < 6000 && i->shipState == THUAI7::ShipState::Attacking) || euclidean_distance(ShipInfo::myself.me.x, ShipInfo::myself.me.y, i->x, i->y)<3000)
                {
                    if (interrupt_codeRecorder.find(ReturnHomeID) == interrupt_codeRecorder.end())
                    {
                        // double angle = i->facingDirection;
                        // double move_angle = angle + PI / 2;
                        api.EndAllAction();
                        // api.Move(300, move_angle);
                        coordinate target = *MapInfo::PositionLists[MapInfo::MyHome].begin();
                        auto end_condition = [](IShipAPI&)
                        { return false; };
                        std::cout << "Triggered RoadSearch,target:" << target.x << "," << target.y << "\n";

                        auto search = std::make_shared<RoadSearch>(target, end_condition);
                        int prior = PRIORITY_ReturnHome * RATIO + callStack.size();
                        callStack.push({*search, ReturnHomeID, prior});
                        interrupt_codeRecorder.insert(ReturnHomeID);
                    }
                }
                
                return;
            }
            else
            {
                //api.EndAllAction();
                if (i->shipState == THUAI7::ShipState::Attacking && euclidean_distance(ShipInfo::myself.me.x, ShipInfo::myself.me.y,i->x,i->y)<6000)
                {
                    if (interrupt_codeRecorder.find(DodgeID) == interrupt_codeRecorder.end() && interrupt_codeRecorder.find(RecoveryID) == interrupt_codeRecorder.end())
                    {
                        api.EndAllAction();
                        double angle = i->facingDirection;
                        coordinate target_ship = {i->x, i->y};
                        coordinate get_pos = judge_attack_pos(api, angle, target_ship);
                        if (get_pos.x<0)
                        {
                            auto init_list = {new BT::eventNode<IShipAPI>{Conditions::JudgeSelfState(THUAI7::ShipState::Idle), ShipAction::AttackFunc(i->x, i->y)}};
                            auto dodgeNode = std::make_shared<BT::SequenceNode<IShipAPI>>(std::move(init_list));
                            interrupt_codeRecorder.insert(DodgeID);
                            int priority = PRIORITY_Dodge * RATIO + callStack.size();
                            callStack.push({[dodgeNode](IShipAPI& api)
                                            { return dodgeNode->perform(api) == BT::SUCCESS; },
                                            DodgeID,
                                            priority});
                        }
                        else
                        {
                            double move_angle = atan2(get_pos.y - ShipInfo::myself.me.y, get_pos.x - ShipInfo::myself.me.x);
                            double time = euclidean_distance(ShipInfo::myself.me.x, ShipInfo::myself.me.y, get_pos.x, get_pos.y)/ShipInfo::myself.me.speed*1000;
                            // BT::SequenceNode<IShipAPI> dodgeNode{
                            auto init_list = {new BT::eventNode<IShipAPI>{Conditions::always_ship, ShipAction::MoveFunc(move_angle, time)}, new BT::eventNode<IShipAPI>{Conditions::JudgeSelfState(THUAI7::ShipState::Idle), ShipAction::AttackFunc(i->x, i->y)}};
                            auto dodgeNode = std::make_shared<BT::SequenceNode<IShipAPI>>(std::move(init_list));
                            interrupt_codeRecorder.insert(DodgeID);
                            int priority = PRIORITY_Dodge * RATIO + callStack.size();
                            callStack.push({[dodgeNode](IShipAPI& api)
                                            { return dodgeNode->perform(api) == BT::SUCCESS; },
                                            DodgeID,
                                            priority});
                        }

                    }
                    break;
                }
                else if (i->shipState == THUAI7::ShipState::Stunned)
                {
                    double dis = euclidean_distance(ShipInfo::myself.me.x, ShipInfo::myself.me.y, i->x, i->y);
                    if (dis < WeaponToDis(ShipInfo::myself.me.weaponType))
                    {
                        api.Attack(atan2(i->y - ShipInfo::myself.me.y, i->x - ShipInfo::myself.me.x));
                    }
                }
                else
                {
                }

            }

        }
    }

    /*
    if (!interrupt_codeRecorder.size())
    {
        for (auto const& i : ShipInfo::Enemies)
        {
            if (i->weaponType!=THUAI7::WeaponType::NullWeaponType&&manhatten_distance(ShipInfo::myself.me.x, ShipInfo::myself.me.y,i->x,i->y)<=6000 )
            {
                interrupt_codeRecorder.push(1);
                callStack.push(detectEnemy);
                break;
            }
        }
    }*/
    auto temp = callStack.top();
    while (temp.func(api))
    {
        std::cout << "Exit!!!!!!,id="<< temp.id << "\n";
		callStack.pop();
        interrupt_codeRecorder.erase(temp.id);
        temp = callStack.top();
        std::cout << "Next id=" << temp.id << "\n";
	}

//    Commute::report(api);
    //ShipStep(api);
    /*
    if (ShipInfo::nearEnemy)
    {
        if (ShipInfo::myself.me.weaponType == THUAI7::WeaponType::NullWeaponType)
        {
            api.EndAllAction();
            return;
        }
    }*/

  //  std::cout << ((nextMode == ATTACK) ? "Next ATTACK\n" : "");
  //  std::cout << ((nextMode == ROADSEARCH) ? "Next ROAD\n" : "");
  //  std::cout << ((nextMode == IDLE) ? "Next IDLE\n" : "");


    
     //if (myself.mode!=cur_Mode)
     //{
     //    shipTreeList[cur_Mode]->Clear();
     //    cur_Mode = myself.mode;

     //}

     //shipTreeList[cur_Mode]->perform(api);
        //std::cout << ShipInfo::myself.me.x/1000 << "  " << ShipInfo::myself.me.y/1000 << ((ShipInfo::myself.mode == ATTACK) ? "  ATTACK\n" : "  NO!!!\n") << ShipInfo::myself.TargetPos.x << "  " << ShipInfo::myself.TargetPos.y<<std::endl;
}

bool ShipStep(IShipAPI& api)
{
    bool res = false;

    if (nextMode == IDLE)
    {
        res = IdleMode::Perform(api);
    }
    else if (nextMode == ATTACK)
    {
        res = AttackMode::Perform(api);
    }
    else if (nextMode == REVENGE)
    {
    }
    else if (nextMode == HIDE)
    {
    }
    else if (nextMode == PRODUCE)
    {
        res = ProduceMode::Perform(api);
    }
    else if (nextMode == CONSTRUCT)
    {
        res = ConstructMode::Perform(api);
    }
    else if (nextMode == ROB)
    {
    }
    else if (nextMode == RUIN)
    {
    }
    else if (nextMode == FOLLOW)
    {
    }
    else if (nextMode==INSPECT)
    {
        res = Inspecting(api);
    }
    if (res)
    {
        ShipStep(api);
    }
    return false;
}

//const char* get_placetype(THUAI7::PlaceType t);




bool run = true;

bool init_root = false;
BT::SequenceNode<ITeamAPI> root;


void AI::play(ITeamAPI& api)  // 默认team playerID 为0
{

    HomeInfo::CheckInfo(api);
    Commute::receive_message(api);
    for (size_t i = 0; i < 4; i++)
    {
        HomeInfo::TeamShipBuffer[i].with_param = false;
    }
    /*
    if (HomeInfo::first_id != 1)
    {
        root.events[0] = new BT::eventNode<ITeamAPI>({Conditions::always, HomeAction::SetShipMode(1 << (HomeInfo::first_id - 1),PRODUCE)});
    }*/
    Commute::process_message();
    if (!HomeInfo::reviveList.empty())
    {
        for (auto iter = HomeInfo::reviveList.begin(); iter != HomeInfo::reviveList.end(); iter++)
        {
            int i = *iter;
            bool res = false;
            if (i <= 2)
            {
                if (api.GetEnergy() >= 4000)
                    res = api.BuildShip(THUAI7::ShipType::CivilianShip, 0).get();
            }
            else
            {
                if (api.GetEnergy() >= 12000)
                    res = api.BuildShip(THUAI7::ShipType::MilitaryShip, 0).get();
            }
            if (res)
            {
                iter = HomeInfo::reviveList.erase(iter);
                if(iter == HomeInfo::reviveList.end())
                {
                    break;
                }
            }
        }
    }
    if (!init_root)
    {
        init_root = true;

        root = {
    new BT::eventNode<ITeamAPI>({Conditions::always, HomeAction::SetShipMode(SHIP_1,PRODUCE, 1)}),
    new BT::eventNode<ITeamAPI>({Conditions::EnergyThreshold(8000), HomeAction::InstallModule(HomeInfo::first_id, THUAI7::ModuleType::ModuleProducer3), Conditions::ShipHasProducer(1, THUAI7::ProducerType::Producer3)}),
    new BT::eventNode<ITeamAPI>({Conditions::EnergyThreshold(12000), HomeAction::BuildShip(THUAI7::ShipType::MilitaryShip), Conditions::ShipAvailable(3)}),
//            new BT::eventNode<ITeamAPI>({Conditions::always, HomeAction::SetShipMode(SHIP_2, ATTACK, MODEPARAM_AttackHome)}),
            new BT::eventNode<ITeamAPI>({Conditions::always, HomeAction::SetShipMode(SHIP_2, IDLE)}), 

    new BT::eventNode<ITeamAPI>({Conditions::EnergyThreshold(4000), HomeAction::BuildShip(THUAI7::ShipType::CivilianShip), Conditions::ShipAvailable(3 - HomeInfo::first_id)}),
    new BT::eventNode<ITeamAPI>({Conditions::always, HomeAction::SetShipMode(SHIP_3, CONSTRUCT, MODEPARAM_ConstructFactory)}),

    new BT::eventNode<ITeamAPI>({Conditions::EnergyThreshold(8000), HomeAction::InstallModule(3 - HomeInfo::first_id, THUAI7::ModuleType::ModuleConstructor3), Conditions::ShipHasConstructor(3 - HomeInfo::first_id, THUAI7::ConstructorType::Constructor3)}),
    new BT::eventNode<ITeamAPI>({Conditions::EnergyThreshold(12000), HomeAction::BuildShip(THUAI7::ShipType::MilitaryShip), Conditions::ShipAvailable(4)}),
    //new BT::eventNode<ITeamAPI>({Conditions::always, HomeAction::SetShipMode(SHIP_4, ATTACK, MODEPARAM_AttackHome)}),


    new BT::eventNode<ITeamAPI>({Conditions::EnergyThreshold(18000), HomeAction::InstallModule(3, THUAI7::ModuleType::ModuleArmor3), Conditions::ShipHasArmor(3, THUAI7::ArmorType::Armor3)}),
    new BT::eventNode<ITeamAPI>({Conditions::EnergyThreshold(18000), HomeAction::InstallModule(3, THUAI7::ModuleType::ModuleShield3), Conditions::ShipHasShield(3, THUAI7::ShieldType::Shield3)}),
    new BT::eventNode<ITeamAPI>({Conditions::EnergyThreshold(10000), HomeAction::InstallModule(HomeInfo::first_id, THUAI7::ModuleType::ModuleLaserGun), Conditions::ShipHasWeapon(1, THUAI7::WeaponType::LaserGun)}),
    new BT::eventNode<ITeamAPI>({Conditions::EnergyThreshold(10000), HomeAction::InstallModule(3 - HomeInfo::first_id, THUAI7::ModuleType::ModuleLaserGun), Conditions::ShipHasWeapon(2, THUAI7::WeaponType::LaserGun)}),
    new BT::eventNode<ITeamAPI>({Conditions::EnergyThreshold(18000), HomeAction::InstallModule(HomeInfo::first_id, THUAI7::ModuleType::ModuleArmor3), Conditions::ShipHasArmor(1, THUAI7::ArmorType::Armor3)}),
    new BT::eventNode<ITeamAPI>({Conditions::EnergyThreshold(18000), HomeAction::InstallModule(HomeInfo::first_id, THUAI7::ModuleType::ModuleShield3), Conditions::ShipHasShield(1, THUAI7::ShieldType::Shield3)}),
    new BT::eventNode<ITeamAPI>({Conditions::EnergyThreshold(18000), HomeAction::InstallModule(3 - HomeInfo::first_id, THUAI7::ModuleType::ModuleArmor3), Conditions::ShipHasArmor(2, THUAI7::ArmorType::Armor3)}),
    new BT::eventNode<ITeamAPI>({Conditions::EnergyThreshold(18000), HomeAction::InstallModule(3 - HomeInfo::first_id, THUAI7::ModuleType::ModuleShield3), Conditions::ShipHasShield(2, THUAI7::ShieldType::Shield3)}),
    new BT::eventNode<ITeamAPI>({Conditions::EnergyThreshold(18000), HomeAction::InstallModule(4, THUAI7::ModuleType::ModuleArmor3), Conditions::ShipHasArmor(4, THUAI7::ArmorType::Armor3)}),
    new BT::eventNode<ITeamAPI>({Conditions::EnergyThreshold(18000), HomeAction::InstallModule(4, THUAI7::ModuleType::ModuleShield3), Conditions::ShipHasShield(4, THUAI7::ShieldType::Shield3)})

};
    }
    root.perform(api);
    Commute::sync_ships(api);
    //std::this_thread::sleep_for(std::chrono::milliseconds(15));
    /*
    new BT::eventNode({Conditions::EnergyThreshold(8000), HomeAction::InstallModule(2, THUAI7::ModuleType::ModuleProducer3)}),
    new BT::eventNode({Conditions::EnergyThreshold(10000), HomeAction::InstallModule(1, THUAI7::ModuleType::ModuleLaserGun)})*/

}
