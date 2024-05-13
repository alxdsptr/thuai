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
#define Instruction_RefreshConstruction (1<<1)


#define Parameter_ResourceRunningOut (1)
#define Parameter_ConstructionBuildUp (1)
#define Parameter_EnemyBuildConstruction (1<<1)
#define Parameter_DestroyedEnemyCunstruction (1<<2)
#define Parameter_DestroyedFriendConstruction (1<<3)


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
    ROADSEARCH
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
        unsigned char ModeParam;
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

    bool UsableShip[5] = {0};
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
}

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
                return Resource;
            case THUAI7::PlaceType::Construction:
                return Construction;
            case THUAI7::PlaceType::Wormhole:
                if (y <= 15 or y >= 35)
                {
                    return ClosedWormhole;
                }
                else
                {
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
            }
        }
    }

}  // namespace MapInfo

namespace ShipInfo
{
    struct ShipMem
    {
        // THUAI7::Ship NearestEnemyShip;
        ShipMode mode=IDLE;
        THUAI7::Ship me;
        coordinate TargetPos = {-1, -1};  //<注意为格子数，不是坐标值
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
};

inline int manhatten_distance(int x1, int y1, int x2, int y2)
{
    return abs(x1 - x2) + abs(y1 - y2);
};
inline int manhatten_distance(int x1, int y1, coordinate c)
{
    return abs(x1 - c.x) + abs(y1 - c.y);
};
double euclidean_distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
};
double WeaponDis[6] = {0, 4000, 4000, 4000, 8000, 8000};
double WeaponToDis(THUAI7::WeaponType p)
{
    return WeaponDis[(int)p];
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
            condition(c),
            action(a),
            state(IDLE),
            judgeSuccess(std::nullopt)
        {}
        eventNode(std::function<bool(T& api)> c, std::function<NodeState(T& api)> a, std::function<bool(T& api)> jf):
            condition(c),
            action(a),
            state(IDLE),
            judgeSuccess(jf)
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

        ~eventNode()
        {
        }
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
        virtual ~baseNode(){}
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

        virtual ~SequenceNode()
        {
            for (size_t i = 0; i < events.size(); i++)
            {
                delete events[i];
                events[i] = NULL;
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
        return [=](IShipAPI& api) {
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
            std::cout << "Send:" << success << std::endl;
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
                if (UsableShip[id] == false)
                {
                    index_to_id[ship_cnt] = id;
                    ship_cnt++;
                }
                temp[id] = true;
            }
            for (int i = 1; i <= 4; i++)
            {
                if (UsableShip[i] and !temp[i])
                {
                    //TODO
                    //设置复活
                }
                UsableShip[i] = temp[i];
            }
            if (init)
            {
                coordinate tmp(MyShips[0].x / 1000, MyShips[0].y/1000);
                /*
                for (size_t i = 0; i < 2; i++)
                {
                    if (manhatten_distance(tmp.x,tmp.y,MapInfo::PositionLists[MapInfo::Home][i])<=2)
                    {
                        MyHomePos = MapInfo::PositionLists[MapInfo::Home][i];
                        EnemyHomePos = MapInfo::PositionLists[MapInfo::Home][1 - i];
                        break;
                    }
                }*/
            }
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
            std::cout << index+1 << "  " << HomeInfo::TeamShipBuffer[index].Mode << std::endl;
            //message[BUFFER_SIZE] = '\0';
            api.SendBinaryMessage(index, message);
        }
    }
    void report(IShipAPI& api)
    {
        if (ShipInfo::ReportBuffer.instruction == 0)
        {
			return;
		}
        std::string message;
        message.resize(REPORTBUFFER_SIZE + 1);
        memcpy(message.data(), &ShipInfo::ReportBuffer, REPORTBUFFER_SIZE);
        api.SendBinaryMessage(0, message);
    }
    std::vector<ReportBuffer> reports;
    void receive_message(ITeamAPI& api)
    {
        while (api.HaveMessage())
        {
            std::string message = api.GetMessage().second;
            Commute::ReportBuffer temp;
            memcpy(&temp, message.data(), REPORTBUFFER_SIZE);
            reports.push_back(std::move(temp));
        }
    }

}


namespace IdleMode
{
    ModeRetval Perform(IShipAPI&)
    {
        if (ShipInfo::myself.mode!=IDLE)
        {
            return {
                ShipInfo::myself.mode,
                true
            };
        }
        return {
            IDLE,
            false
        };
    }

    void Clear(IShipAPI& api)
    {

    }
}

namespace AttackMode
{

}

namespace RoadSearchMode
{
    std::deque<coordinate> path;
    bool visited[maxLen][maxLen];
    coordinate from[maxLen][maxLen];


    /**
    * @brief 寻路终止条件
    */
    std::function<bool(IShipAPI& api)> end_condition = [](IShipAPI& api)
    {
        if (euclidean_distance(ShipInfo::myself.me.x, ShipInfo::myself.me.y, ShipInfo::myself.TargetPos.x, ShipInfo::myself.TargetPos.y) 
                <= WeaponToDis(ShipInfo::myself.me.weaponType) + 200 
            and api.HaveView(ShipInfo::myself.TargetPos.x, ShipInfo::myself.TargetPos.y))
        {
            return true;
        }
        else
        {
            return false;
        }
    };


    auto is_empty = [](int x,int y)
    {
        MapInfo::Place t = MapInfo::fullmap[x][y];
        return (t == MapInfo::Space or t == MapInfo::Shadow or t == MapInfo::OpenWormhole);
        //THUAI7::PlaceType t = MapInfo::map[x][y];
        //return (t == THUAI7::PlaceType::Space or t == THUAI7::PlaceType::Shadow or t == THUAI7::PlaceType::Wormhole);
    };
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
                    //if (i == 0 and j == 0)
                    //    continue;
                    //if (i == -1 and j == -1 and (!is_empty(MapInfo::fullmap[now.x - 1][now.y]) or !is_empty(MapInfo::fullmap[now.x][now.y - 1])))
                    //    continue;
                    //if (i == 1 and j == -1 and (!is_empty(MapInfo::fullmap[now.x + 1][now.y]) or !is_empty(MapInfo::fullmap[now.x][now.y - 1])))
                    //    continue;
                    //if (i == -1 and j == 1 and (!is_empty(MapInfo::fullmap[now.x - 1][now.y]) or !is_empty(MapInfo::fullmap[now.x][now.y + 1])))
                    //    continue;
                    //if (i == 1 and j == 1 and (!is_empty(MapInfo::fullmap[now.x + 1][now.y]) or !is_empty(MapInfo::fullmap[now.x][now.y + 1])))
                    //    continue;
                    if (i == 0 and j == 0)
                        continue;
                    if (i == -1 and j == -1 and (!is_empty(now.x - 1,now.y) or !is_empty(now.x,now.y - 1)))
                        continue;
                    if (i == 1 and j == -1 and (!is_empty(now.x + 1,now.y) or !is_empty(now.x,now.y - 1)))
                        continue;
                    if (i == -1 and j == 1 and (!is_empty(now.x - 1,now.y) or !is_empty(now.x,now.y + 1)))
                        continue;
                    if (i == 1 and j == 1 and (!is_empty(now.x + 1,now.y) or !is_empty(now.x,now.y + 1)))
                        continue;
                    int nx = now.x + i, ny = now.y + j;
                    if (nx < 0 or nx >= 50 or ny < 0 or ny >= 50 or visited[nx][ny])
                        continue;
                    if (nx == x2 and ny == y2)
                    {
                        double cost = sqrt(i * i + j * j);
                        pq.push({nx, ny, coordinate{now.x, now.y}, now.cost + cost, 0});
                    }
                    //if ((t != MapInfo::Space and t != MapInfo::Shadow and t != MapInfo::OpenWormhole) || (coordinate(nx, ny) == *MapInfo::NoStep.find(coordinate(nx, ny))))
                    if (!is_empty(nx, ny))
                        continue;
                    double cost = sqrt(i * i + j * j);
                    int h = manhatten_distance(nx, ny, x2, y2);
                    pq.push({nx, ny, coordinate{now.x, now.y}, now.cost + cost, (double)h});
                }
            }
        }
    }

    /**
     * @brief 已经弃用，请勿使用
     */
    std::deque<coordinate> search_road(int x1, int y1, THUAI7::PlaceType type, IShipAPI& api)
    {
        int min_dis = 0x7fffffff;
        auto index = MapInfo::PlaceTypeConvert(type, 0, 0);
        auto& des = MapInfo::PositionLists[index];
        for (auto i = des.begin(); i != des.end();)
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
            else if (type == THUAI7::PlaceType::Construction and api.GetConstructionState((*i).x, (*i).y).second > 0)
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
            for (int i = -1; i <= 1; i++)
            {
                for (int j = -1; j <= 1; j++)
                {
                    if (i == 0 and j == 0)
                        continue;
                    if (i == -1 and j == -1 and (!is_empty(now.x - 1,now.y) or !is_empty(now.x,now.y - 1)))
                        continue;
                    if (i == 1 and j == -1 and (!is_empty(now.x + 1,now.y) or !is_empty(now.x,now.y - 1)))
                        continue;
                    if (i == -1 and j == 1 and (!is_empty(now.x - 1,now.y) or !is_empty(now.x,now.y + 1)))
                        continue;
                    if (i == 1 and j == 1 and (!is_empty(now.x + 1,now.y) or !is_empty(now.x,now.y + 1)))
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
                    if (!is_empty(nx, ny))
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

    ModeRetval Perform(IShipAPI& api)
    {
        if (path.empty())
        {
            if (manhatten_distance(ShipInfo::myself.me.x / 1000, ShipInfo::myself.me.y / 1000, ShipInfo::myself.TargetPos) <= 1)
            {
                return {
                    IDLE,
                    true
                };
            }
            path = search_road(ShipInfo::myself.me.x / 1000, ShipInfo::myself.me.y / 1000, ShipInfo::myself.TargetPos.x, ShipInfo::myself.TargetPos.y);
        }
        if (end_condition(api))
        {
            path.clear();
            return {
                IDLE,
                true
            };
        }
        if (api.GetSelfInfo()->shipState == THUAI7::ShipState::Idle and !path.empty())
        {
            //bool direct_move = true;
            std::vector<std::pair<coordinate, MapInfo::Place>> temp;
            THUAI7::Ship& me = ShipInfo::myself.me;
            for (auto const& ship : ShipInfo::FriendShips)
            {
                int x = ship->x / 1000, y = ship->y / 1000, lx = ship->x % 1000, ly = ship->y % 1000;
                int mx = ShipInfo::myself.me.x / 1000, my = ShipInfo::myself.me.y / 1000;
                auto saveAndChange = [&temp](int x, int y)
                {
                    temp.push_back({{x, y}, MapInfo::fullmap[x][y]});
                    MapInfo::fullmap[x][y] = MapInfo::Place::Ruin;
                };
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
            }
            /*
            for (size_t i = 0; i < ShipInfo::FriendShips.size(); i++)
            {
                if (euclidean_distance(ShipInfo::myself.me.x / 1000, ShipInfo::myself.me.y / 1000, ShipInfo::FriendShips[i]->x / 1000, ShipInfo::FriendShips[i]->y / 1000)<1.5)
                {
                    direct_move = false;
                    coordinate tmp_co(ShipInfo::FriendShips[i]->x / 1000, ShipInfo::FriendShips[i]->y / 1000);
                    tmplist.insert(tmp_co);
                    MapInfo::NoStep.insert(tmp_co);
                }
            }
            if (direct_move)
            {
                api.Move(time, angle);
            }
            else*/
            if(!temp.empty())
                path = search_road(ShipInfo::myself.me.x / 1000, ShipInfo::myself.me.y / 1000, ShipInfo::myself.TargetPos.x, ShipInfo::myself.TargetPos.y);
            for (const auto & i : temp)
            {
                MapInfo::fullmap[i.first.x][i.first.y] = i.second;
            }
            auto next = path.front();
            path.pop_front();
            int dx = next.x * 1000 + 500 - ShipInfo::myself.me.x;
            int dy = next.y * 1000 + 500 - ShipInfo::myself.me.y;
            std::cout << "x: " << ShipInfo::myself.me.x << " y: " << ShipInfo::myself.me.y << "nx: " << next.x << "ny: " << next.y << std::endl;
            std::cout << "dx: " << dx << " dy: " << dy << std::endl;
            double time = sqrt(dx * dx + dy * dy) / 3.0;
            double angle = atan2(dy, dx);
            std::cout << "time: " << time << " angle: " << angle << std::endl;
            auto res = api.Move(time, angle);
            if (!res.get())
            {
                std::cout << "move fail"; 
            }
        }
        return {
            ROADSEARCH,
            false
        };
    }

    void Clear(IShipAPI& api)
    {
        path.clear();
    }

}
namespace ConstructMode
{

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
        ShipInfo::myself.TargetPos = tmp;
        return true;
    }

    ModeRetval Perform(IShipAPI& api)
    {
        //if (euclidean_distance(ShipInfo::myself.me.x / 1000, ShipInfo::myself.me.y / 1000, ShipInfo::myself.TargetPos.x, ShipInfo::myself.TargetPos.y) <= 1.5)
        auto near_enough = [](IShipAPI& api)
        {
            //return (manhatten_distance(ShipInfo::myself.me.x / 1000, ShipInfo::myself.me.y / 1000, ShipInfo::myself.TargetPos.x, ShipInfo::myself.TargetPos.y) <= 1);
            return (euclidean_distance(ShipInfo::myself.me.x / 1000, ShipInfo::myself.me.y / 1000, ShipInfo::myself.TargetPos.x, ShipInfo::myself.TargetPos.y) <= 1.5);
        };
        if (ShipInfo::myself.TargetPos.x==-1)
        {
            if (GetNearestConstruction(api))
            {
                /* [](IShipAPI& api)
            {
            return (manhatten_distance(ShipInfo::myself.me.x / 1000, ShipInfo::myself.me.y / 1000, ShipInfo::myself.TargetPos.x, ShipInfo::myself.TargetPos.y) <= 1);
            };*/

                RoadSearchMode::end_condition = near_enough;
                return {
                    ROADSEARCH,
                    true
                };
            }
            else
            {
                return {
                    IDLE,
                    false
                };
            }
        }
        if (near_enough(api))
        {
            auto res = api.GetConstructionState(ShipInfo::myself.TargetPos.x, ShipInfo::myself.TargetPos.y);
            if (res.second <= 0)
            {
                if (ShipInfo::myself.me.shipState != THUAI7::ShipState::Constructing)
                {
                    api.EndAllAction();
                    api.Construct(ShipInfo::constructType);
                }
                return
                {
                    CONSTRUCT,
                    false
                };
            }
            else
            {
                MapInfo::PositionLists[MapInfo::Construction].erase(ShipInfo::myself.TargetPos);
                ShipInfo::ReportBuffer = {Instruction_RefreshConstruction, Parameter_ConstructionBuildUp, ShipInfo::myself.TargetPos};
                Commute::report(api);
                ShipInfo::myself.TargetPos = {-1, -1};

                    if (GetNearestConstruction(api))
                    {
                            /* [](IShipAPI& api)
                        {
                        return (manhatten_distance(ShipInfo::myself.me.x / 1000, ShipInfo::myself.me.y / 1000, ShipInfo::myself.TargetPos.x, ShipInfo::myself.TargetPos.y) <= 1);
                        };*/

                        RoadSearchMode::end_condition = near_enough;
                        return {
                            ROADSEARCH,
                            true
                        };
                    }
                    else
                    {
                        return {
                            IDLE,
                            false
                        };
                    }
            }
        }
        else
        {
            if (GetNearestConstruction(api))
            {
                return {
                    ROADSEARCH,
                    true
                };
            }
            else
            {
                return {
                    IDLE,
                    false
                };
            }
        }
    }

    void Clear(IShipAPI& api)
    {

    }
}

namespace ProduceMode
{
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
        ShipInfo::myself.TargetPos = tmp;
        return true;
    }

    ModeRetval Perform(IShipAPI& api)
    {
        //if (euclidean_distance(ShipInfo::myself.me.x / 1000, ShipInfo::myself.me.y / 1000, ShipInfo::myself.TargetPos.x, ShipInfo::myself.TargetPos.y) <= 1.5)
        auto near_enough = [](IShipAPI& api)
        {
            //return (manhatten_distance(ShipInfo::myself.me.x / 1000, ShipInfo::myself.me.y / 1000, ShipInfo::myself.TargetPos.x, ShipInfo::myself.TargetPos.y) <= 1);
            return (euclidean_distance(ShipInfo::myself.me.x / 1000, ShipInfo::myself.me.y / 1000, ShipInfo::myself.TargetPos.x, ShipInfo::myself.TargetPos.y) <= 1.5);
        };
        if (ShipInfo::myself.TargetPos.x == -1)
        {
            if (GetNearestResource(api))
            {
                RoadSearchMode::end_condition = near_enough;
                return {
                    ROADSEARCH,
                    true
                };
            }
            else
            {
                return {
                    IDLE,
                    false
                };
            }
        }
        if (near_enough(api))
        {
            if (api.GetResourceState(ShipInfo::myself.TargetPos.x, ShipInfo::myself.TargetPos.y))
            {
                if (ShipInfo::myself.me.shipState != THUAI7::ShipState::Producing)
                {
                    api.EndAllAction();
                    api.Produce();
                }

                return {
                    PRODUCE,
                    false
                };
            }
            else
            {
                MapInfo::PositionLists[MapInfo::Resource].erase(ShipInfo::myself.TargetPos);
                ShipInfo::ReportBuffer = {Instruction_RefreshResource, Parameter_ResourceRunningOut, ShipInfo::myself.TargetPos};
                Commute::report(api);
                ShipInfo::myself.TargetPos = {-1, -1};

                    if (GetNearestResource(api))
                    {
                        RoadSearchMode::end_condition = near_enough;
                        return {
                            ROADSEARCH,
                            true
                        };
                    }
                    else
                    {
                        return {
                            IDLE,
                            false
                        };
                    }



            }

        }
        else
        {
            if (GetNearestResource(api))
            {
                return {
                    ROADSEARCH,
                    true
                };
            }
            else
            {
                return {
                    IDLE,
                    false
                };
            }
        }
    }

    void Clear(IShipAPI& api)
    {

    }
}

namespace AttackMode
{

    int getHp(IShipAPI& api,coordinate tar)
    {
        THUAI7::PlaceType tmp = MapInfo::map[tar.x][tar.y];
        switch (tmp)
        {
            case THUAI7::PlaceType::NullPlaceType:
                return 0;
                break;
            case THUAI7::PlaceType::Home:
                return (MapInfo::MySide == RED) ? (api.GetGameInfo()->blueHomeHp) : (api.GetGameInfo()->redHomeHp);
                break;
            case THUAI7::PlaceType::Space:
                return 0;
                break;
            case THUAI7::PlaceType::Ruin:
                return 0;
                break;
            case THUAI7::PlaceType::Shadow:
                return 0;
                break;
            case THUAI7::PlaceType::Asteroid:
                return 0;
                break;
            case THUAI7::PlaceType::Resource:
                return 0;
                break;
            case THUAI7::PlaceType::Construction:
                return api.GetConstructionState(tar.x, tar.y).second;
                break;
            case THUAI7::PlaceType::Wormhole:
                return api.GetWormholeHp(tar.x, tar.y);
                break;
            default:
                break;
        }
    }
    ModeRetval Perform(IShipAPI& api)
    {
        if (euclidean_distance(ShipInfo::myself.me.x, ShipInfo::myself.me.y, ShipInfo::myself.TargetPos.x*1000+500, ShipInfo::myself.TargetPos.y*1000+500) <= WeaponToDis(ShipInfo::myself.me.weaponType) + 200)
        {
            if (getHp(api,ShipInfo::myself.TargetPos)==0)
            {
                return {
                    IDLE,
                    false
                };
            }
            if (ShipInfo::myself.me.shipState != THUAI7::ShipState::Idle)
            {
                api.EndAllAction();
            }

            double angle = atan2(ShipInfo::myself.TargetPos.y * 1000 + 500 - ShipInfo::myself.me.y, ShipInfo::myself.TargetPos.x * 1000 + 500 - ShipInfo::myself.me.x);
            //double angle = -0.3;
            std::cout << "mydirect: " << ShipInfo::myself.me.facingDirection << std::endl;
            std::cout << "angle: " << angle << std::endl;
            api.Attack(angle);
            return {
                ATTACK,
                false
            };
        }
        else
        {
            RoadSearchMode::end_condition = [](IShipAPI& api)
            {
                std::cout << "endcondition\n";
                if (euclidean_distance(ShipInfo::myself.me.x, ShipInfo::myself.me.y, ShipInfo::myself.TargetPos.x * 1000 + 500, ShipInfo::myself.TargetPos.y * 1000 + 500) <= WeaponToDis(ShipInfo::myself.me.weaponType) + 200 && api.HaveView(ShipInfo::myself.TargetPos.x*1000+500, ShipInfo::myself.TargetPos.y*1000+500)) /*api.HaveView(ShipInfo::myself.TargetPos.x, ShipInfo::myself.TargetPos.y)*/
                {
                    return true;
                }
                else
                {
                    return false;
                }
            };
            return {
                ROADSEARCH,
                true
            };
        }
    }

}



//ModeRetval (*perform_list[3])(IShipAPI&) = {&(IdleMode::Perform), &(RoadSearchMode::Perform), &(ProduceMode::Perform)};
void (*clear_list[3])(IShipAPI&) = {&(IdleMode::Clear), &(RoadSearchMode::Clear), &(ProduceMode::Clear)};



ShipMode nextMode=IDLE;


void ShipStep(IShipAPI &api);


enum additionalMode
{
    Normal,
    Run,
    DodgeBullets
};


std::stack<std::function<void(IShipAPI&)>> callStack;
//{&ShipStep};

auto WrapperFunc(std::function<BT::NodeState(IShipAPI& api)> func)
{
    return [=](IShipAPI& api)
    {
        if (func(api) == BT::NodeState::SUCCESS)
        {
            callStack.pop();
        }
    };
}
int Ship_Init = 1;
void clear(IShipAPI& api)
{
    for (auto const& clear : clear_list)
    {
       clear(api);
    }
}

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

/**
 * @brief 判断伤害大小
 * @param weapon 武器类型
 * @param towhich 0表示本体，1表示装甲，2表示护盾
 * @return 
 */
inline int WeaponToDamage(THUAI7::WeaponType weapon, unsigned char towhich)
{
    int damage=0, mag=0;
    switch (weapon)
    {
        case THUAI7::WeaponType::NullWeaponType:
            damage = 0, mag = 0;
            break;
        case THUAI7::WeaponType::LaserGun:
            damage = 1200, mag = (towhich == 0) ? 1 : (towhich == 1 ? 1.5 : 0.6);
            break;
        case THUAI7::WeaponType::PlasmaGun:
            damage = 1300, mag = (towhich == 0) ? 1 : (towhich == 1 ? 2 : 0.4);
            break;
        case THUAI7::WeaponType::ShellGun:
            damage = 1800, mag = (towhich == 0) ? 1 : (towhich == 1 ? 0.4 : 1.5);
            break;
        case THUAI7::WeaponType::MissileGun:
            damage = 1600, mag = (towhich == 0) ? 1 : (towhich == 1 ? 1 : 1000);
            break;
        case THUAI7::WeaponType::ArcGun:
            damage = 3200, mag = (towhich == 0) ? 1 : (towhich == 1 ? 2 : 2);
            break;
        default:
            break;
    }
    return damage * mag;
}

double WeaponHarm[6][3] = {
    {1200, 1.5, 0.6},
    {1300, 2, 0.4},
    {1800, 0.4, 1.5},
    {1600, 1, 1000},
    {1800, 2, 2}
};
struct health
{
    int hp;
    int armor;
    int shield;

    void operator-=(THUAI7::WeaponType a)
    {
        /*
        if (a == THUAI7::WeaponType::MissileGun)
        {
            if (armor>0)
            {
                int dam = WeaponToDamage(a, 1);
                armor = (dam > armor) ? 0 : (armor - dam);
            }
            else if (hp>0)
            {
                int dam = WeaponToDamage(a, 0);
                hp = (dam > hp) ? 0 : (hp - dam);
            }
        }
        else
        {
            if (shield>0)
            {
                int dam = WeaponToDamage(a, 2);
                shield = (dam > shield) ? 0 : (shield - dam);
            }
            else if (armor > 0)
            {
                int dam = WeaponToDamage(a, 1);
                armor = (dam > armor) ? 0 : (armor - dam);
            }
            else if (hp > 0)
            {
                int dam = WeaponToDamage(a, 0);
                hp = (dam > hp) ? 0 : (hp - dam);
            }
        }*/
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


std::stack<unsigned char> interrupt_codeRecorder;


/**
 * @brief 对应的Interruptcode:1
 */
auto detectEnemy=[](IShipAPI& api)
{ 
        std::cout << "Detected Enemies!!!\n";
       
    if (!ShipInfo::Enemies.size())
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
};



void AI::play(IShipAPI& api)
{
    ShipInfo::CheckInfo(api);
    if (Ship_Init)
    {
        nextMode = IDLE;
        callStack.push(&ShipStep);
        Ship_Init = 0;
    }
    if (Commute::RefreshInfo(api))
    {
        std::cout << "Success\n"
                  << ((ShipInfo::ShipBuffer.Mode == PRODUCE) ? "PRODUCE\n" : "NO!!!\n");
        
        if (ShipInfo::myself.mode!=ShipInfo::ShipBuffer.Mode)
        {
            clear(api);
            switch (ShipInfo::ShipBuffer.Mode)
            {
                case PRODUCE:
                    if (ShipInfo::ShipBuffer.with_target)
                    {
                        ShipInfo::myself.TargetPos = ShipInfo::ShipBuffer.target;
                        
                    }
                    nextMode = ShipInfo::myself.mode = PRODUCE;
                    break;
                case CONSTRUCT:
                    nextMode = ShipInfo::myself.mode = CONSTRUCT;
                    if (ShipInfo::ShipBuffer.with_target)
                    {
                        ShipInfo::myself.TargetPos = ShipInfo::ShipBuffer.target;
                    }
                    ShipInfo::myself.mode = CONSTRUCT;
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
                    nextMode = ShipInfo::myself.mode = ATTACK;
                    switch (ShipInfo::ShipBuffer.ModeParam)
                    {
                        case MODEPARAM_AttackHome:
                            ShipInfo::myself.TargetPos = *(MapInfo::PositionLists[MapInfo::EnemyHome].begin());
                            break;
                        default:
                            ShipInfo::myself.TargetPos = ShipInfo::ShipBuffer.target;
                            break;
                    }
                    
                    break;
                default:
                    break;
            }
        }
        if (ShipInfo::ShipBuffer.with_param)
        {
            switch (ShipInfo::ShipBuffer.instruction)
            {
                case Instruction_RefreshResource:
                    switch (ShipInfo::ShipBuffer.param)
                    {
                        case Parameter_ResourceRunningOut:
                            MapInfo::PositionLists[MapInfo::Resource].erase(ShipInfo::ShipBuffer.param_pos);
                            break;
                        default:
                            break;
                    }
                    break;
                case Instruction_RefreshConstruction:
                    switch (ShipInfo::ShipBuffer.param)
                    {
                        case Parameter_ConstructionBuildUp:
                            MapInfo::PositionLists[MapInfo::Resource].erase(ShipInfo::ShipBuffer.param_pos);
                            break;
                        case Parameter_EnemyBuildConstruction:
                            if (ShipInfo::myself.me.shipType==THUAI7::ShipType::MilitaryShip)
                            {
                            }
                            else if (ShipInfo::myself.me.shipType == THUAI7::ShipType::CivilianShip)
                            {
                            }
                            else
                            {
                            }
                            break;
                        case Parameter_DestroyedEnemyCunstruction:
                            MapInfo::PositionLists[MapInfo::Resource].insert(ShipInfo::ShipBuffer.param_pos);
                            break;
                        case Parameter_DestroyedFriendConstruction:
                            MapInfo::PositionLists[MapInfo::Resource].insert(ShipInfo::ShipBuffer.param_pos);
                            break;
                        default:
                            break;
                    }
                    break;
                default:
                    break;
            }
        }
    }

    for (auto const& i : ShipInfo::Enemies)
    {
        if (i->shipState == THUAI7::ShipState::Attacking)
        {
            api.EndAllAction();
            double angle = i->facingDirection;
            double move_angle = angle + PI / 2;
            // BT::SequenceNode<IShipAPI> dodgeNode{
            auto dodgeNode = new BT::SequenceNode<IShipAPI>{
                new BT::eventNode<IShipAPI>{Conditions::always_ship, ShipAction::MoveFunc(move_angle, 200)},
                new BT::eventNode<IShipAPI>{Conditions::JudgeSelfState(THUAI7::ShipState::Idle), ShipAction::AttackFunc(i->x, i->y)},
            };

            callStack.push(WrapperFunc([dodgeNode](IShipAPI& api)
                                       { return dodgeNode->perform(api); }));
            break;
        }

    }

    //if (!interrupt_codeRecorder.size())
    //{
    //    for (auto const& i : ShipInfo::Enemies)
    //    {
    //        if (i->weaponType!=THUAI7::WeaponType::NullWeaponType&&manhatten_distance(ShipInfo::myself.me.x, ShipInfo::myself.me.y,i->x,i->y)<=6000 && abs(atan2(ShipInfo::myself.me.y - i->y, ShipInfo::myself.me.x - i->x) - i->facingDirection) <= PI / 36)
    //        {
    //            interrupt_codeRecorder.push(1);
    //            callStack.push(detectEnemy);
    //            break;
    //        }
    //    }
    //}
    std::cout << ShipInfo::myself.mode << std::endl;
    callStack.top()(api);

    Commute::report(api);
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

void ShipStep(IShipAPI& api)
{
    ModeRetval res;
    res.immediate = false;
    if (nextMode == IDLE)
    {
        res = IdleMode::Perform(api);
        nextMode = res.mode;
    }
    else if (nextMode == ATTACK)
    {
        std::cout << "mydirect: " << ShipInfo::myself.me.facingDirection << std::endl;
        res = AttackMode::Perform(api);
        nextMode = res.mode;
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
        nextMode = res.mode;
    }
    else if (nextMode == CONSTRUCT)
    {
        res = ConstructMode::Perform(api);
        nextMode = res.mode;
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
    else if (nextMode == ROADSEARCH)
    {
        res = RoadSearchMode::Perform(api);
        nextMode = res.mode;
    }
    if (res.immediate)
    {
        ShipStep(api);
    }
}

//const char* get_placetype(THUAI7::PlaceType t);
bool hasSend = false;
bool hasInstall = false;
bool hasBuild = false;
bool BuildSecondCivil = false;

volatile int Ind = 2;



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
    if (!Commute::reports.empty())
    {
        for (size_t i = 0; i < 4; i++)
        {
            HomeInfo::TeamShipBuffer[i].with_param = true;
        }
        Commute::ReportBuffer tmp = *Commute::reports.begin();
        Commute::reports.erase(Commute::reports.begin());
        switch (tmp.instruction)
        {
            case Instruction_RefreshResource:
                switch (tmp.param)
                {
                    case Parameter_ResourceRunningOut:
                        for (size_t i = 0; i < 4; i++)
                        {
                            HomeInfo::TeamShipBuffer[i].with_param = 1;
                            HomeInfo::TeamShipBuffer[i].instruction = Instruction_RefreshResource;
                            HomeInfo::TeamShipBuffer[i].param = Parameter_ResourceRunningOut;
                            HomeInfo::TeamShipBuffer[i].param_pos = tmp.param_pos;
                        }
                        break;
                    default:
                        break;
                }
                break;
            case Instruction_RefreshConstruction:
                switch (ShipInfo::ShipBuffer.param)
                {
                    case Parameter_ConstructionBuildUp:
                        for (size_t i = 0; i < 4; i++)
                        {
                            HomeInfo::TeamShipBuffer[i].with_param = 1;
                            HomeInfo::TeamShipBuffer[i].instruction = Instruction_RefreshConstruction;
                            HomeInfo::TeamShipBuffer[i].param = Parameter_ConstructionBuildUp;
                            HomeInfo::TeamShipBuffer[i].param_pos = tmp.param_pos;
                        }
                        break;
                    case Parameter_EnemyBuildConstruction:
                        for (size_t i = 0; i < 4; i++)
                        {
                            HomeInfo::TeamShipBuffer[i].with_param = 1;
                            HomeInfo::TeamShipBuffer[i].instruction = Instruction_RefreshConstruction;
                            HomeInfo::TeamShipBuffer[i].param = Parameter_EnemyBuildConstruction;
                            HomeInfo::TeamShipBuffer[i].param_pos = tmp.param_pos;
                        }
                        break;
                    case Parameter_DestroyedEnemyCunstruction:
                        for (size_t i = 0; i < 4; i++)
                        {
                            HomeInfo::TeamShipBuffer[i].with_param = 1;
                            HomeInfo::TeamShipBuffer[i].instruction = Instruction_RefreshConstruction;
                            HomeInfo::TeamShipBuffer[i].param = Parameter_DestroyedEnemyCunstruction;
                            HomeInfo::TeamShipBuffer[i].param_pos = tmp.param_pos;
                        }
                        break;
                    case Parameter_DestroyedFriendConstruction:
                        for (size_t i = 0; i < 4; i++)
                        {
                            HomeInfo::TeamShipBuffer[i].with_param = 1;
                            HomeInfo::TeamShipBuffer[i].instruction = Instruction_RefreshConstruction;
                            HomeInfo::TeamShipBuffer[i].param = Parameter_DestroyedFriendConstruction;
                            HomeInfo::TeamShipBuffer[i].param_pos = tmp.param_pos;
                        }
                        break;
                    default:
                        break;
                }
                break;
            default:
                break;
        }

    }
    /*
    if (HomeInfo::first_id != 1)
    {
        root.events[0] = new BT::eventNode<ITeamAPI>({Conditions::always, HomeAction::SetShipMode(1 << (HomeInfo::first_id - 1),PRODUCE)});
    }*/
    if (!init_root)
    {
        init_root = true;
        root = {
    new BT::eventNode<ITeamAPI>({Conditions::always, HomeAction::SetShipMode(SHIP_1,PRODUCE)}),
    new BT::eventNode<ITeamAPI>({Conditions::EnergyThreshold(8000), HomeAction::InstallModule(HomeInfo::first_id, THUAI7::ModuleType::ModuleProducer3), Conditions::ShipHasProducer(1, THUAI7::ProducerType::Producer3)}),
    new BT::eventNode<ITeamAPI>({Conditions::EnergyThreshold(12000), HomeAction::BuildShip(THUAI7::ShipType::MilitaryShip), Conditions::ShipAvailable(3)}),
    new BT::eventNode<ITeamAPI>({Conditions::always, HomeAction::SetShipMode(SHIP_2, ATTACK, MODEPARAM_AttackHome)}), 

    new BT::eventNode<ITeamAPI>({Conditions::EnergyThreshold(4000), HomeAction::BuildShip(THUAI7::ShipType::CivilianShip), Conditions::ShipAvailable(3 - HomeInfo::first_id)}),
    new BT::eventNode<ITeamAPI>({Conditions::always, HomeAction::SetShipMode(SHIP_3, CONSTRUCT, MODEPARAM_ConstructFactory)}),

    new BT::eventNode<ITeamAPI>({Conditions::EnergyThreshold(8000), HomeAction::InstallModule(3 - HomeInfo::first_id, THUAI7::ModuleType::ModuleConstructor3), Conditions::ShipHasConstructor(2, THUAI7::ConstructorType::Constructor3)}),
    new BT::eventNode<ITeamAPI>({Conditions::EnergyThreshold(12000), HomeAction::BuildShip(THUAI7::ShipType::MilitaryShip), Conditions::ShipAvailable(4)}),
    new BT::eventNode<ITeamAPI>({Conditions::always, HomeAction::SetShipMode(SHIP_4, ATTACK, MODEPARAM_AttackHome)}),
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

/*
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
                if (i == -1 and j == -1 and (!is_empty(MapInfo::map[now.x - 1][now.y]) or !is_empty(MapInfo::map[now.x][now.y - 1])))
                    continue;
                if (i == 1 and j == -1 and (!is_empty(MapInfo::map[now.x + 1][now.y]) or !is_empty(MapInfo::map[now.x][now.y - 1])))
                    continue;
                if (i == -1 and j == 1 and (!is_empty(MapInfo::map[now.x - 1][now.y]) or !is_empty(MapInfo::map[now.x][now.y + 1])))
                    continue;
                if (i == 1 and j == 1 and (!is_empty(MapInfo::map[now.x + 1][now.y]) or !is_empty(MapInfo::map[now.x][now.y + 1])))
                    continue;
                int nx = now.x + i, ny = now.y + j;
                if (nx < 0 or nx >= 50 or ny < 0 or ny >= 50 or visited[nx][ny])
                    continue;
                if (nx == x2 and ny == y2)
                {
                    double cost = sqrt(i * i + j * j);
                    pq.push({nx, ny, coordinate{now.x, now.y}, now.cost + cost, 0});
                }
                THUAI7::PlaceType t = MapInfo::map[nx][ny];
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
    int min_dis = 0x7fffffff, index = MapInfo::getIndex(type);
    auto& des = MapInfo::des_list[index];
    for (auto i = des.begin(); i != des.end();)
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
                if (i == -1 and j == -1 and (!is_empty(MapInfo::map[now.x - 1][now.y]) or !is_empty(MapInfo::map[now.x][now.y - 1])))
                    continue;
                if (i == 1 and j == -1 and (!is_empty(MapInfo::map[now.x + 1][now.y]) or !is_empty(MapInfo::map[now.x][now.y - 1])))
                    continue;
                if (i == -1 and j == 1 and (!is_empty(MapInfo::map[now.x - 1][now.y]) or !is_empty(MapInfo::map[now.x][now.y + 1])))
                    continue;
                if (i == 1 and j == 1 and (!is_empty(MapInfo::map[now.x + 1][now.y]) or !is_empty(MapInfo::map[now.x][now.y + 1])))
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
*/
/*
    **
     * @brief 节点功能：反复执行子节点，直到返回SUCCESS
     *
     *
    class TryUntilSuccessNode : public baseNode
    {
    public:
        baseNode* child;
        TryUntilSuccessNode(baseNode* x) :
            baseNode(RUNNING),
            child(x)
        {
        }

        /**
         * @brief TryUntilSuccess对应的perform虚函数
         * @param api
         * @return 若子节点返回SUCCESS或当前状态已经为SUCCESS，则返回SUCCESS；反之返回RUNNING
         *
        virtual NodeState perform(ITeamAPI& api)
        {
            if (state == IDLE)
            {
                state = RUNNING;
            }
            else if (state == RUNNING)
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

    /**
     * @brief decorator节点，执行一次子节点，不论子节点返回值如何，均返回SUCCESS
     
    class AlwaysSuccessNode : public baseNode
    {
    public:
        baseNode* child;  ///< 要执行的子节点
        AlwaysSuccessNode(baseNode* x) :
            baseNode(IDLE),
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

    /**
     * @brief 选择器节点，依次尝试每个子节点，直到有一个成功或全部失败
     */
    /*
    class fallbackNode : public baseNode
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
        std::vector<baseNode*> events;
        int curChild;
*/
        /**
         * @brief 选择器节点的perform函数
         * - 如果当前子节点返回SUCCESS，则重置全部子节点状态，本节点状态设为SUCCESS并返回SUCCESS
         * - 如果当前子节点返回RUNNING，下一次调用时仍调用该子节点，直到它返回FAIL或SUCCESS；返回RUNNING
         * - 如果当前子节点返回FAIL
         *  . 如果已经是最后一个子节点，则本节点状态为FAIL，返回FAIL
         *  . 否则下一次调用执行下一个子节点，本次返回RUNNING
         * @param api
         * @return
     */
    /*
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
                case FAIL:
                    if (curChild == events.size() - 1)
                    {
                        state = FAIL;
                        ResetChildren();
                    }
                    else
                    {
                        curChild++;
                    }
                    break;
                case SUCCESS:
                    state = SUCCESS;
                    ResetChildren();
                    break;
                default:
                    break;
            }
            return state;
        }

        fallbackNode(std::initializer_list<baseNode*> l) :
            events(l),
            curChild(0)
        {
        }

        fallbackNode(const fallbackNode& a) :
            curChild(0)
        {
            for (size_t i = 0; i < a.events.size(); i++)
            {
                events.push_back(new auto(*a.events[i]));
            }
        }

        virtual ~fallbackNode()
        {
            for (size_t i = 0; i < events.size(); i++)
            {
                delete events[i];
                events[i] = NULL;
            }
        }
    };*/
