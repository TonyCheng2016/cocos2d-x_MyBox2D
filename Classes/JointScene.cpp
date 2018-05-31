#include "JointScene.h"
#include "cocostudio/CocoStudio.h"
#include "ui/CocosGUI.h"

USING_NS_CC;

#define MAX_2(X,Y) (X)>(Y) ? (X) : (Y)

#define StaticAndDynamicBodyExample 1
using namespace cocostudio::timeline;

JointScene::~JointScene()
{

#ifdef BOX2D_DEBUG
	if (_DebugDraw != NULL) delete _DebugDraw;
#endif

	if (_b2World != nullptr) delete _b2World;
//  for releasing Plist&Texture
//	SpriteFrameCache::getInstance()->removeSpriteFramesFromFile("box2d.plist");
	Director::getInstance()->getTextureCache()->removeUnusedTextures();

}

Scene* JointScene::createScene()
{
    auto scene = Scene::create();
    auto layer = JointScene::create();
    scene->addChild(layer);
    return scene;
}

// on "init" you need to initialize your instance
bool JointScene::init()
{   
    //////////////////////////////
    // 1. super init first
    if ( !Layer::init() )
    {
        return false;
    }

//  For Loading Plist+Texture
	SpriteFrameCache::getInstance()->addSpriteFramesWithFile("box2d.plist");

	_visibleSize = Director::getInstance()->getVisibleSize();
	Vec2 origin = Director::getInstance()->getVisibleOrigin();
    
	//標題 : 顯示目前 BOX2D 所介紹的功能
	_titleLabel = Label::createWithTTF("Joint Scene", "fonts/Marker Felt.ttf", 32);
	_titleLabel->setPosition(_titleLabel->getContentSize().width*0.5f + 25.f, _visibleSize.height - _titleLabel->getContentSize().height*0.5f - 5.f);
	this->addChild(_titleLabel, 2);

	// 建立 Box2D world
	_b2World = nullptr;
	b2Vec2 Gravity = b2Vec2(0.0f, -9.8f);		//重力方向
	bool AllowSleep = true;			//允許睡著
	_b2World = new b2World(Gravity);	//創建世界
	_b2World->SetAllowSleeping(AllowSleep);	//設定物件允許睡著

											// Create Scene with csb file
	_csbRoot = CSLoader::createNode("JointScene.csb");
#ifndef BOX2D_DEBUG
	// 設定顯示背景圖示
	auto bgSprite = _csbRoot->getChildByName("bg64_1");
	bgSprite->setVisible(true);

#endif
	addChild(_csbRoot, 1);

	createStaticBoundary();
	setupMouseJoint();
	setupDistanceJoint();
	setupPrismaticJoint();
	setupPulleyJoint();
	setupGearJoint();
	setupWeldJoint();
	//setupRopeJoint();
	setStaticWalls();
	setupDesnity();
	setupSensor();
#ifdef BOX2D_DEBUG
	//DebugDrawInit
	_DebugDraw = nullptr;
	_DebugDraw = new GLESDebugDraw(PTM_RATIO);
	//設定DebugDraw
	_b2World->SetDebugDraw(_DebugDraw);
	//選擇繪製型別
	uint32 flags = 0;
	flags += GLESDebugDraw::e_shapeBit;						//繪製形狀
	flags += GLESDebugDraw::e_pairBit;						
	flags += GLESDebugDraw::e_jointBit;
	flags += GLESDebugDraw::e_centerOfMassBit;
	flags += GLESDebugDraw::e_aabbBit;
	//設定繪製類型
	_DebugDraw->SetFlags(flags);
#endif
	_b2World->SetContactListener(&_contactListener);

	_listener1 = EventListenerTouchOneByOne::create();	//創建一個一對一的事件聆聽器
	_listener1->onTouchBegan = CC_CALLBACK_2(JointScene::onTouchBegan, this);		//加入觸碰開始事件
	_listener1->onTouchMoved = CC_CALLBACK_2(JointScene::onTouchMoved, this);		//加入觸碰移動事件
	_listener1->onTouchEnded = CC_CALLBACK_2(JointScene::onTouchEnded, this);		//加入觸碰離開事件

	this->_eventDispatcher->addEventListenerWithSceneGraphPriority(_listener1, this);	//加入剛創建的事件聆聽器
	this->schedule(CC_SCHEDULE_SELECTOR(JointScene::doStep));

    return true;
}

void JointScene::setupMouseJoint()
{
	// 取得並設定 frame01 畫框圖示為動態物件
	auto frameSprite = _csbRoot->getChildByName("frame01");
	Point loc = frameSprite->getPosition();

	b2BodyDef bodyDef;
	bodyDef.type = b2_dynamicBody;
	bodyDef.position.Set(loc.x / PTM_RATIO, loc.y / PTM_RATIO);
	bodyDef.userData = frameSprite;
	b2Body *body = _b2World->CreateBody(&bodyDef);

	// Define poly shape for our dynamic body.
	b2PolygonShape rectShape;
	Size frameSize = frameSprite->getContentSize();
	rectShape.SetAsBox((frameSize.width - 4)*0.5f / PTM_RATIO, (frameSize.height - 4)*0.5f / PTM_RATIO);
	// Define the dynamic body fixture.
	b2FixtureDef fixtureDef;
	fixtureDef.shape = &rectShape;
	fixtureDef.restitution = 0.1f;
	fixtureDef.density = 1.0f;
	fixtureDef.friction = 0.1f;
	body->CreateFixture(&fixtureDef);


	// 取得並設定 carBody 畫框圖示為動態物件
	auto carBodySprite = (Sprite *)_csbRoot->getChildByName("carBody");
	Point carloc;
	carloc = carBodySprite->getPosition();
	bodyDef.position.Set(carloc.x / PTM_RATIO, carloc.y / PTM_RATIO);
	bodyDef.userData = carBodySprite;
	_carBodybody = _b2World->CreateBody(&bodyDef);

	// Define poly shape for our dynamic body.
	frameSize = carBodySprite->getContentSize();
	rectShape.SetAsBox((frameSize.width - 100)*0.5f / PTM_RATIO, (frameSize.height - 60)*0.5f / PTM_RATIO);
	// Define the dynamic body fixture.
	fixtureDef.restitution = 0.1f;
	fixtureDef.density = 1.0f;
	fixtureDef.friction = 0.1f;
	_carBodybody->CreateFixture(&fixtureDef);

	// 取得並設定 circle01 wheel_1為【動態物體】
	auto wheel_1circleSprite = _csbRoot->getChildByName("wheel_1");
	loc = wheel_1circleSprite->getPosition();
	frameSize = wheel_1circleSprite->getContentSize();
	b2CircleShape circleShape;
	circleShape.m_radius = frameSize.width*0.5f / PTM_RATIO;

	b2BodyDef dynamicBodyDef;
	dynamicBodyDef.type = b2_dynamicBody;
	dynamicBodyDef.position.Set(loc.x / PTM_RATIO, (loc.y+10) / PTM_RATIO);
	dynamicBodyDef.userData = wheel_1circleSprite;
	b2Body* wheel_1dynamicBody = _b2World->CreateBody(&dynamicBodyDef);

	fixtureDef.shape = &circleShape;
	wheel_1dynamicBody->CreateFixture(&fixtureDef);


	// 取得並設定 circle01 wheel_2為【動態物體】
	auto wheel_2circleSprite = _csbRoot->getChildByName("wheel_2");
	loc = wheel_2circleSprite->getPosition();
	frameSize = wheel_2circleSprite->getContentSize();

	dynamicBodyDef.position.Set(loc.x / PTM_RATIO, (loc.y ) / PTM_RATIO);
	dynamicBodyDef.userData = wheel_2circleSprite;
	b2Body* wheel_2dynamicBody = _b2World->CreateBody(&dynamicBodyDef);

	fixtureDef.shape = &circleShape;
	wheel_2dynamicBody->CreateFixture(&fixtureDef);

	// 建立 Joint 連結
	b2RevoluteJointDef carJointwheel_1;
	carJointwheel_1.bodyA = _carBodybody;
	carJointwheel_1.localAnchorA.Set(-0.7f, -0.7f);
	carJointwheel_1.bodyB = wheel_1dynamicBody;
	carJointwheel_1.localAnchorB.Set(0, 0);
	_b2World->CreateJoint(&carJointwheel_1);

	b2RevoluteJointDef carJointwheel_2;
	carJointwheel_2.bodyA = _carBodybody;
	carJointwheel_2.localAnchorA.Set(0.9f, -0.7f);
	carJointwheel_2.bodyB = wheel_2dynamicBody;
	carJointwheel_2.localAnchorB.Set(0, 0);
	_b2World->CreateJoint(&carJointwheel_2);


	////產生距離關節
	//b2DistanceJointDef JointDef;
	//JointDef.Initialize(carBodybody, wheel_1dynamicBody, carBodybody->GetPosition(), wheel_1dynamicBody->GetPosition());
	//_b2World->CreateJoint(&JointDef);

	////產生距離關節
	////b2DistanceJointDef JointDef;
	//JointDef.Initialize(carBodybody, wheel_2dynamicBody, carBodybody->GetPosition(), wheel_2dynamicBody->GetPosition());
	//_b2World->CreateJoint(&JointDef);

	_bTouchOn = false;

	_contactListener.setCollisionTarget(*carBodySprite);
	_tdelayTime = 0; // 第一次一定可以噴發
	_bSparking = true; // 可以噴發




					   //取得並設定 circle01_rope 為【動態物體】
	auto ropecircleSprite = _csbRoot->getChildByName("circle01_rope");
	Point locTail = ropecircleSprite->getPosition();
	Size sizeTail = ropecircleSprite->getContentSize();

	bodyDef.type = b2_dynamicBody;
	bodyDef.position.Set(locTail.x / PTM_RATIO, locTail.y / PTM_RATIO);
	bodyDef.userData = ropecircleSprite;
	b2Body* donutsBody = _b2World->CreateBody(&bodyDef);
	b2CircleShape ropecircleShape;
	ropecircleShape.m_radius = (sizeTail.width - 4)*0.5f / PTM_RATIO;
	fixtureDef.shape = &ropecircleShape;
	fixtureDef.density = 0.01f;  fixtureDef.friction = 0.25f; fixtureDef.restitution = 0.25f;
	donutsBody->CreateFixture(&fixtureDef);


	//產生繩子關節
	b2RopeJointDef JointDef;
	JointDef.bodyA = _carBodybody;
	JointDef.bodyB = donutsBody;
	JointDef.localAnchorA = b2Vec2(0, 0);
	JointDef.localAnchorB = b2Vec2(0, 30.0f / PTM_RATIO);
	JointDef.maxLength = (carloc.y - locTail.y - 30) / PTM_RATIO;
	JointDef.collideConnected = true;
	b2RopeJoint* J = (b2RopeJoint*)_b2World->CreateJoint(&JointDef);

	// 中間再以線段相連，
	char tmp[20] = "";
	Sprite *ropeSprite[14];
	Point loc1[14];
	Size  size[15];
	b2Body* ropeBody[14];

	b2PolygonShape boxShape;
	bodyDef.type = b2_dynamicBody;
	// 因為是繩子所以重量不要太重
	fixtureDef.density = 0.01f;  fixtureDef.friction = 1.0f; fixtureDef.restitution = 0.0f;
	fixtureDef.shape = &boxShape;
	// 產生一系列的繩子段落 rope01_01 ~ rope01_15，同時接起來
	for (int i = 0; i < 14; i++)
	{
		sprintf(tmp, "rope01_%02d", i + 1);
		ropeSprite[i] = (Sprite *)_csbRoot->getChildByName(tmp);
		loc1[i] = ropeSprite[i]->getPosition();
		size[i] = ropeSprite[i]->getContentSize();

		bodyDef.position.Set(loc1[i].x / PTM_RATIO, loc1[i].y / PTM_RATIO);
		bodyDef.userData = ropeSprite[i];
		ropeBody[i] = _b2World->CreateBody(&bodyDef);
		boxShape.SetAsBox((size[i].width - 4)*0.5f / PTM_RATIO, (size[i].height - 4)*0.5f / PTM_RATIO);
		ropeBody[i]->CreateFixture(&fixtureDef);
	}
	// 利用 RevoluteJoint 將線段彼此連接在一起
	// 先連結 ropeHeadBody 與  ropeBody[0]

	float locAnchor = 0.5f*(size[0].height - 5) / PTM_RATIO;
	b2RevoluteJointDef revJoint;
	revJoint.bodyA = _carBodybody;
	revJoint.localAnchorA.Set(0, -0.9f);
	revJoint.bodyB = ropeBody[0];
	revJoint.localAnchorB.Set(0, locAnchor);
	_b2World->CreateJoint(&revJoint);
	for (int i = 0; i < 13; i++) {
		revJoint.bodyA = ropeBody[i];
		revJoint.localAnchorA.Set(0, -locAnchor);
		revJoint.bodyB = ropeBody[i + 1];
		revJoint.localAnchorB.Set(0, locAnchor);
		_b2World->CreateJoint(&revJoint);
	}
	revJoint.bodyA = ropeBody[13];
	revJoint.localAnchorA.Set(0, -locAnchor);
	revJoint.bodyB = donutsBody;
	revJoint.localAnchorB.Set(0, 0.85f);
	_b2World->CreateJoint(&revJoint);
}


void JointScene::setupDistanceJoint()
{
	// 取得並設定 frame01_dist 畫框圖示為【靜態物體】
	auto frameSprite = _csbRoot->getChildByName("frame01_dist");
	Point loc = frameSprite->getPosition();
	Size size = frameSprite->getContentSize();
	b2BodyDef staticBodyDef;
	staticBodyDef.type = b2_staticBody;
	staticBodyDef.position.Set(loc.x / PTM_RATIO, loc.y / PTM_RATIO);
	staticBodyDef.userData = frameSprite;
	b2Body* staticBody = _b2World->CreateBody(&staticBodyDef);

	b2PolygonShape boxShape;
	boxShape.SetAsBox(size.width*0.5f / PTM_RATIO, size.height*0.5f / PTM_RATIO);

	b2FixtureDef fixtureDef;
	fixtureDef.shape = &boxShape;
	staticBody->CreateFixture(&fixtureDef);

	// 取得並設定 circle01 為【動態物體】
	auto circleSprite = _csbRoot->getChildByName("circle01_dist");
	loc = circleSprite->getPosition();
	size = circleSprite->getContentSize();
	b2CircleShape circleShape;
	circleShape.m_radius = size.width*0.5f / PTM_RATIO;

	b2BodyDef dynamicBodyDef;
	dynamicBodyDef.type = b2_dynamicBody;
	dynamicBodyDef.position.Set(loc.x / PTM_RATIO, loc.y / PTM_RATIO);
	dynamicBodyDef.userData = circleSprite;
	b2Body* dynamicBody = _b2World->CreateBody(&dynamicBodyDef);
	fixtureDef.shape = &circleShape;
	dynamicBody->CreateFixture(&fixtureDef);

	//產生距離關節
	b2DistanceJointDef JointDef;
	JointDef.Initialize(staticBody, dynamicBody, staticBody->GetPosition(), dynamicBody->GetPosition());
	_b2World->CreateJoint(&JointDef);
}

void JointScene::setupPrismaticJoint()
{
	// 取得並設定 frame01_pri 畫框圖示為【靜態物體】
	auto frameSprite = _csbRoot->getChildByName("frame01_pri");
	Point loc = frameSprite->getPosition();
	Size size = frameSprite->getContentSize();
	b2BodyDef staticBodyDef;
	staticBodyDef.type = b2_staticBody;
	staticBodyDef.position.Set(loc.x / PTM_RATIO, loc.y / PTM_RATIO);
	staticBodyDef.userData = frameSprite;
	b2Body* staticBody = _b2World->CreateBody(&staticBodyDef);
	b2PolygonShape boxShape;
	boxShape.SetAsBox(size.width*0.5f / PTM_RATIO, size.height*0.5f / PTM_RATIO);
	b2FixtureDef fixtureDef;
	fixtureDef.shape = &boxShape;
	staticBody->CreateFixture(&fixtureDef);

	// 取得並設定 circle01_pri 為【動態物體】
	auto circleSprite = _csbRoot->getChildByName("circle01_pri");
	loc = circleSprite->getPosition();
	size = circleSprite->getContentSize();
	float scale = circleSprite->getScale();
	b2CircleShape circleShape;
	circleShape.m_radius = size.width*0.5f*scale / PTM_RATIO;

	b2BodyDef dynamicBodyDef;
	dynamicBodyDef.type = b2_dynamicBody;
	dynamicBodyDef.position.Set(loc.x / PTM_RATIO, loc.y / PTM_RATIO);
	dynamicBodyDef.userData = circleSprite;
	b2Body* dynamicBody = _b2World->CreateBody(&dynamicBodyDef);
	fixtureDef.shape = &circleShape;
	dynamicBody->CreateFixture(&fixtureDef);

	//產生垂直 Prismatic Joint
	b2PrismaticJointDef JointDef;
	JointDef.Initialize(staticBody, dynamicBody, staticBody->GetPosition(), b2Vec2(1.0f / PTM_RATIO, 0));
	_b2World->CreateJoint(&JointDef);

	// 取得並設定 circle02_pri 為【動態物體】
	circleSprite = _csbRoot->getChildByName("circle02_pri");
	loc = circleSprite->getPosition();
	size = circleSprite->getContentSize();
	scale = circleSprite->getScale();
	circleShape.m_radius = size.width*0.5f*scale / PTM_RATIO;

	dynamicBodyDef.type = b2_dynamicBody;
	dynamicBodyDef.position.Set(loc.x / PTM_RATIO, loc.y / PTM_RATIO);
	dynamicBodyDef.userData = circleSprite;
	dynamicBody = _b2World->CreateBody(&dynamicBodyDef);
	fixtureDef.shape = &circleShape;
	fixtureDef.density = 50.0f;
	dynamicBody->CreateFixture(&fixtureDef);

	//產生水平 Prismatic Joint
	JointDef;
	JointDef.Initialize(staticBody, dynamicBody, staticBody->GetPosition(), b2Vec2(0, 1.0f / PTM_RATIO));
	_b2World->CreateJoint(&JointDef);
}

// 滑輪關節
void JointScene::setupPulleyJoint()
{
	// 取得並設定 circle01_pul 為【動態物體A】
	auto circleSprite = _csbRoot->getChildByName("circle01_pul");
	Point locA = circleSprite->getPosition();
	Size size = circleSprite->getContentSize();
	float scale = circleSprite->getScale();
	b2CircleShape circleShape;
	circleShape.m_radius = size.width*0.5f*scale / PTM_RATIO;

	b2BodyDef bodyDef;
	bodyDef.type = b2_dynamicBody;
	bodyDef.position.Set(locA.x / PTM_RATIO, locA.y / PTM_RATIO);
	bodyDef.userData = circleSprite;
	b2Body* bodyA = _b2World->CreateBody(&bodyDef);
	b2FixtureDef fixtureDef;
	fixtureDef.shape = &circleShape;
	fixtureDef.density = 10.0f;
	fixtureDef.friction = 0.2f;
	bodyA->CreateFixture(&fixtureDef);

	// 取得並設定 circle02_pul 為【動態物體B】
	circleSprite = _csbRoot->getChildByName("circle02_pul");
	Point locB = circleSprite->getPosition();
	size = circleSprite->getContentSize();
	scale = circleSprite->getScale();
	circleShape.m_radius = size.width*0.5f*scale / PTM_RATIO;

	bodyDef.type = b2_dynamicBody;
	bodyDef.position.Set(locB.x / PTM_RATIO, locB.y / PTM_RATIO);
	bodyDef.userData = circleSprite;
	b2Body* bodyB = _b2World->CreateBody(&bodyDef);
	fixtureDef.shape = &circleShape;
	fixtureDef.density = 10.0f;
	fixtureDef.friction = 0.2f;
	bodyB->CreateFixture(&fixtureDef);

	//產生滑輪關節
	b2PulleyJointDef JointDef;
	JointDef.Initialize(bodyA, bodyB,
		b2Vec2( locA.x / PTM_RATIO, (locA.y +150) / PTM_RATIO),
		b2Vec2( locB.x / PTM_RATIO, (locB.y +150) / PTM_RATIO),
		bodyA->GetWorldCenter(),
		bodyB->GetWorldCenter(),
		1);
	_b2World->CreateJoint(&JointDef);
}

// 本範例共有六個會動的動態物體，五個是圓形一個是長方形
void JointScene::setupGearJoint()
{
	// 首先產生六組不顯示的【圓形】靜態物體，以固定可以轉動的六個動態物體
	// 因為六個動態物體已經在螢幕上，可以先取得該 Sprite ，同時取得座標
	char tmp[20] = "";
	Sprite *gearSprite[6];
	Point loc[6];
	Size  size[6];
	float scale[6];
	b2Body* staticBody[6];
	b2Body* dynamicBody[6];
	b2RevoluteJoint*  RevJoint[5];
	b2PrismaticJoint* PriJoint;

	b2BodyDef staticBodyDef;
	staticBodyDef.type = b2_staticBody;
	staticBodyDef.userData = NULL;

	b2CircleShape staticShape;
	staticShape.m_radius = 5 / PTM_RATIO;
	b2FixtureDef fixtureDef;
	fixtureDef.shape = &staticShape;

	// 建立六個靜態的圓形 Body
	// 同時建立六個動態的Body，以綁住 gear01_01 ~  gear01_06 的圖示
	for (int i = 0; i <  6; i++)
	{
		sprintf(tmp, "gear01_%02d", i+1);
		gearSprite[i] = (Sprite *)_csbRoot->getChildByName(tmp);
		loc[i] = gearSprite[i]->getPosition();
		size[i] = gearSprite[i]->getContentSize();
		scale[i] = gearSprite[i]->getScale();

		staticBodyDef.position.Set(loc[i].x / PTM_RATIO, loc[i].y / PTM_RATIO);
		staticBody[i] = _b2World->CreateBody(&staticBodyDef);
		staticBody[i]->CreateFixture(&fixtureDef);
	}

	b2BodyDef dynamicBodyDef;
	dynamicBodyDef.type = b2_dynamicBody;

	b2CircleShape circleShape;
	b2PolygonShape polyShape;
	fixtureDef.shape = &circleShape;
	fixtureDef.density = 1.0f;
	fixtureDef.friction = 0.2f;
	fixtureDef.restitution = 0.25f;
	// 第六個是矩形要另外處裡
	for (int i = 0; i < 6; i++)
	{
		if (i < 5) circleShape.m_radius = (size[i].width-10) * 0.5f * scale[i] / PTM_RATIO;
		else {
			float sx = gearSprite[i]->getScaleX();
			float sy = gearSprite[i]->getScaleY();
			fixtureDef.shape = &polyShape;
			polyShape.SetAsBox((size[i].width-4) *0.5f *sx / PTM_RATIO, (size[i].height-4) *0.5f *sy / PTM_RATIO);
		}
		dynamicBodyDef.userData = gearSprite[i];
		dynamicBodyDef.position.Set(loc[i].x / PTM_RATIO, loc[i].y / PTM_RATIO);
		dynamicBody[i] = _b2World->CreateBody(&dynamicBodyDef);
		dynamicBody[i]->CreateFixture(&fixtureDef);
	}

	b2RevoluteJointDef RJoint;	// 旋轉關節
	b2PrismaticJointDef PrJoint; // 平移關節
	for (int i = 0; i < 6; i++)
	{
		if (i < 5) {
			RJoint.Initialize(staticBody[i], dynamicBody[i], dynamicBody[i]->GetWorldCenter());
			RevJoint[i] = (b2RevoluteJoint*)_b2World->CreateJoint(&RJoint);
		}
		else {
			PrJoint.Initialize(staticBody[i], dynamicBody[i], dynamicBody[i]->GetWorldCenter(), b2Vec2(1.0f, 0));
			PriJoint = (b2PrismaticJoint*)_b2World->CreateJoint(&PrJoint);
		}
	}
	//產生齒輪關節(同步相反)
	b2GearJointDef GJoint;
	GJoint.bodyA = dynamicBody[0];
	GJoint.bodyB = dynamicBody[1];
	GJoint.joint1 = RevJoint[0];
	GJoint.joint2 = RevJoint[1];
	GJoint.ratio = 1;
	_b2World->CreateJoint(&GJoint);
	//產生齒輪關節(A 為 B 的兩倍快同向)
	GJoint.bodyA = dynamicBody[2];
	GJoint.bodyB = dynamicBody[3];
	GJoint.joint1 = RevJoint[2];
	GJoint.joint2 = RevJoint[3];
	GJoint.ratio = -2;
	_b2World->CreateJoint(&GJoint);
	//產生齒輪關節(同步同向)
	GJoint.bodyA = dynamicBody[4];
	GJoint.bodyB = dynamicBody[5];
	GJoint.joint1 = RevJoint[4];
	GJoint.joint2 = PriJoint;
	GJoint.ratio = -1;
	_b2World->CreateJoint(&GJoint);
}

void JointScene::setupWeldJoint()
{
	// 取得並設定 frame01_weld 為【靜態物體】
	auto frameSprite = _csbRoot->getChildByName("frame01_weld");
	Point loc = frameSprite->getPosition();
	Size size = frameSprite->getContentSize();
	b2BodyDef staticBodyDef;
	staticBodyDef.type = b2_staticBody;
	staticBodyDef.position.Set(loc.x / PTM_RATIO, loc.y / PTM_RATIO);
	staticBodyDef.userData = frameSprite;
	b2Body* staticBody = _b2World->CreateBody(&staticBodyDef);
	b2PolygonShape boxShape;
	boxShape.SetAsBox(size.width*0.5f / PTM_RATIO, size.height*0.5f / PTM_RATIO);
	b2FixtureDef fixtureDef;
	fixtureDef.shape = &boxShape;
	staticBody->CreateFixture(&fixtureDef);

	//取得並設定 circle01_weld 為【動態物體】
	auto circleSprite = _csbRoot->getChildByName("circle01_weld");
	loc = circleSprite->getPosition();
	size = circleSprite->getContentSize();

	b2BodyDef dynamicBodyDef;
	dynamicBodyDef.type = b2_dynamicBody;
	dynamicBodyDef.position.Set(loc.x / PTM_RATIO, loc.y / PTM_RATIO);
	dynamicBodyDef.userData = circleSprite;
	b2Body* dynamicBody1 = _b2World->CreateBody(&dynamicBodyDef);
	b2CircleShape circleShape;
	circleShape.m_radius = (size.width - 4)*0.5f / PTM_RATIO;
	fixtureDef.shape = &circleShape;
	fixtureDef.density = 1.0f;  fixtureDef.friction = 0.25f; fixtureDef.restitution = 0.25f;
	dynamicBody1->CreateFixture(&fixtureDef);

	//取得並設定 frame02_weld 為【動態物體】
	frameSprite = _csbRoot->getChildByName("frame02_weld");
	loc = frameSprite->getPosition();
	size = frameSprite->getContentSize();

	dynamicBodyDef.position.Set(loc.x / PTM_RATIO, loc.y / PTM_RATIO);
	dynamicBodyDef.userData = frameSprite;
	b2Body* dynamicBody2 = _b2World->CreateBody(&dynamicBodyDef);
	boxShape.SetAsBox(size.width*0.5f / PTM_RATIO, size.height*0.5f / PTM_RATIO);
	fixtureDef.shape = &boxShape;
	fixtureDef.density = 1.0f;  fixtureDef.friction = 0.25f; fixtureDef.restitution = 0.25f;
	dynamicBody2->CreateFixture(&fixtureDef);

	////產生焊接關節(固定)
	b2WeldJointDef JointDef;
	JointDef.Initialize(staticBody, dynamicBody2, staticBody->GetPosition() + b2Vec2(-30 / PTM_RATIO, -30 / PTM_RATIO));
	_b2World->CreateJoint(&JointDef); // 使用預設值焊接

	//產生焊接關節(可轉動)
	JointDef.Initialize(staticBody, dynamicBody1, staticBody->GetPosition() + b2Vec2(30 / PTM_RATIO, 30 / PTM_RATIO));
	JointDef.frequencyHz = 1.0f;
	JointDef.dampingRatio = 0.125f;
	_b2World->CreateJoint(&JointDef);
}

void JointScene::setupRopeJoint()
{
	// 取得並設定 frame01_rope 畫框圖示為【靜態物體】
	auto frameSprite = _csbRoot->getChildByName("frame01_rope");
	Point locHead = frameSprite->getPosition();
	Size sizeHead = frameSprite->getContentSize();

	b2BodyDef bodyDef;
	bodyDef.type = b2_dynamicBody;
	bodyDef.position.Set(locHead.x / PTM_RATIO, locHead.y / PTM_RATIO);
	bodyDef.userData = frameSprite;
	b2Body* ropeHeadBody = _b2World->CreateBody(&bodyDef);
	b2FixtureDef  fixtureDef;
	fixtureDef.density = 1.0f;  fixtureDef.friction = 0.25f; fixtureDef.restitution = 0.25f;
	b2PolygonShape boxShape;
	boxShape.SetAsBox(sizeHead.width*0.5f / PTM_RATIO, sizeHead.height*0.5f / PTM_RATIO);
	fixtureDef.shape = &boxShape;
	ropeHeadBody->CreateFixture(&fixtureDef);

	//取得並設定 circle01_rope 為【動態物體】
	auto ropecircleSprite = _csbRoot->getChildByName("circle01_rope");
	Point locTail = ropecircleSprite->getPosition();
	Size sizeTail = ropecircleSprite->getContentSize();

	bodyDef.type = b2_dynamicBody;
	bodyDef.position.Set(locTail.x / PTM_RATIO, locTail.y / PTM_RATIO);
	bodyDef.userData = ropecircleSprite;
	b2Body* donutsBody = _b2World->CreateBody(&bodyDef);
	b2CircleShape ropecircleShape;
	ropecircleShape.m_radius = (sizeTail.width - 4)*0.5f / PTM_RATIO;
	fixtureDef.shape = &ropecircleShape;
	fixtureDef.density = 0.01f;  fixtureDef.friction = 0.25f; fixtureDef.restitution = 0.25f;
	donutsBody->CreateFixture(&fixtureDef);


	//產生繩子關節
	b2RopeJointDef JointDef;
	JointDef.bodyA = ropeHeadBody;
	JointDef.bodyB = donutsBody;
	JointDef.localAnchorA = b2Vec2(0, 0);
	JointDef.localAnchorB = b2Vec2(0, 30.0f / PTM_RATIO);
	JointDef.maxLength = (locHead.y - locTail.y- 30)/PTM_RATIO;
	JointDef.collideConnected = true;
	b2RopeJoint* J = (b2RopeJoint*)_b2World->CreateJoint(&JointDef);

	// 中間再以線段相連，
	char tmp[20] = "";
	Sprite *ropeSprite[14];
	Point loc[14];
	Size  size[15];
	b2Body* ropeBody[14];

	bodyDef.type = b2_dynamicBody;
	// 因為是繩子所以重量不要太重
	fixtureDef.density = 0.01f;  fixtureDef.friction = 1.0f; fixtureDef.restitution =0.0f;
	fixtureDef.shape = &boxShape;
	// 產生一系列的繩子段落 rope01_01 ~ rope01_15，同時接起來
	for (int i = 0; i < 14; i++)
	{
		sprintf(tmp, "rope01_%02d", i + 1);
		ropeSprite[i] = (Sprite *)_csbRoot->getChildByName(tmp);
		loc[i] = ropeSprite[i]->getPosition();
		size[i] = ropeSprite[i]->getContentSize();

		bodyDef.position.Set(loc[i].x / PTM_RATIO, loc[i].y / PTM_RATIO);
		bodyDef.userData = ropeSprite[i];
		ropeBody[i] = _b2World->CreateBody(&bodyDef);
		boxShape.SetAsBox((size[i].width-4)*0.5f / PTM_RATIO, (size[i].height-4)*0.5f / PTM_RATIO);
		ropeBody[i]->CreateFixture(&fixtureDef);
	}
	// 利用 RevoluteJoint 將線段彼此連接在一起
	// 先連結 ropeHeadBody 與  ropeBody[0]

	float locAnchor = 0.5f*(size[0].height - 5) / PTM_RATIO;
	b2RevoluteJointDef revJoint;
	revJoint.bodyA = ropeHeadBody;
	revJoint.localAnchorA.Set(0, -0.9f);
	revJoint.bodyB = ropeBody[0];
	revJoint.localAnchorB.Set(0, locAnchor);
	_b2World->CreateJoint(&revJoint);
	for (int i = 0; i < 13; i++) {
		revJoint.bodyA = ropeBody[i];
		revJoint.localAnchorA.Set(0, -locAnchor);
		revJoint.bodyB = ropeBody[i+1];
		revJoint.localAnchorB.Set(0, locAnchor);
		_b2World->CreateJoint(&revJoint);
	}
	revJoint.bodyA = ropeBody[13];
	revJoint.localAnchorA.Set(0, -locAnchor);
	revJoint.bodyB = donutsBody;
	revJoint.localAnchorB.Set(0, 0.85f);
	_b2World->CreateJoint(&revJoint);
}

void JointScene::setStaticWalls()
{
	b2BodyDef bodyDef;
	bodyDef.type = b2_staticBody; // 設定這個 Body 為 靜態的
	bodyDef.userData = NULL;
	// 在 b2World 中產生該 Body, 並傳回產生的 b2Body 物件的指標
	// 產生一次，就可以讓後面所有的 Shape 使用
	b2Body *staticbody = _b2World->CreateBody(&bodyDef);

	// 產生靜態邊界所需要的 EdgeShape，名稱都是 wall 為開頭
	b2EdgeShape edgeShape;
	b2FixtureDef fixtureDef; // 產生 Fixture
	fixtureDef.shape = &edgeShape;
	fixtureDef.friction = 0.1f;
	char tmp[20] = "";
	Point pntLoc = _csbRoot->getPosition();

	for (size_t i = 1; i <= 54; i++)
	{
		// 產生所需要的 Sprite file name int plist 
		// 此處取得的都是相對於 csbRoot 所在位置的相對座標
		// 在計算 edgeShape 的相對應座標時，必須進行轉換
		sprintf(tmp, "wall1_%02d", i);
		auto edgeSprite = (Sprite *)_csbRoot->getChildByName(tmp);
		Size ts = edgeSprite->getContentSize();
		Point loc = edgeSprite->getPosition();
		float angle = edgeSprite->getRotation();
		float scale = edgeSprite->getScaleX();	// 水平的線段圖示假設都只有對 X 軸放大

		Point lep1, lep2, wep1, wep2; // EdgeShape 的兩個端點
		lep1.y = 0; lep1.x = -(ts.width - 4) / 2.0f;
		lep2.y = 0; lep2.x = (ts.width - 4) / 2.0f;

		// 所有的線段圖示都是是本身的中心點為 (0,0)，
		// 根據縮放、旋轉產生所需要的矩陣
		// 根據寬度計算出兩個端點的座標，然後呈上開矩陣
		// 然後進行旋轉，
		// Step1: 先CHECK 有無旋轉，有旋轉則進行端點的計算
		cocos2d::Mat4 modelMatrix, rotMatrix;
		modelMatrix.m[0] = scale;  // 先設定 X 軸的縮放
		cocos2d::Mat4::createRotationZ(angle*M_PI / 180.0f, &rotMatrix);
		modelMatrix.multiply(rotMatrix);
		modelMatrix.m[3] = pntLoc.x + loc.x; //設定 Translation，自己的加上父親的
		modelMatrix.m[7] = pntLoc.y + loc.y; //設定 Translation，自己的加上父親的

											 // 產生兩個端點
		wep1.x = lep1.x * modelMatrix.m[0] + lep1.y * modelMatrix.m[1] + modelMatrix.m[3];
		wep1.y = lep1.x * modelMatrix.m[4] + lep1.y * modelMatrix.m[5] + modelMatrix.m[7];
		wep2.x = lep2.x * modelMatrix.m[0] + lep2.y * modelMatrix.m[1] + modelMatrix.m[3];
		wep2.y = lep2.x * modelMatrix.m[4] + lep2.y * modelMatrix.m[5] + modelMatrix.m[7];

		// bottom edge
		edgeShape.Set(b2Vec2(wep1.x / PTM_RATIO, wep1.y / PTM_RATIO), b2Vec2(wep2.x / PTM_RATIO, wep2.y / PTM_RATIO));
		staticbody->CreateFixture(&fixtureDef);
	}

}
void JointScene::setupDesnity()
{
	Point pntLoc = _csbRoot->getPosition();

	b2BodyDef bodyDef;
	bodyDef.userData = NULL;
	// 在 b2World 中產生該 Body, 並傳回產生的 b2Body 物件的指標
	// 產生一次，就可以讓後面所有的 Shape 使用
	b2FixtureDef fixtureDef; // 產生 Fixture
	char tmp[20] = "";

	// 產生三角形靜態物體所需要的 triShape
	// 產生蹺蹺板底座的三角形
	b2Body *seesawBasedbody;
	b2PolygonShape triShape;
	fixtureDef.shape = &triShape;
	fixtureDef.density = 1.0f;
	fixtureDef.friction = 0.2f;
	fixtureDef.restitution = 0.8f;
	for (size_t i = 1; i <= 1; i++)
	{
		// 產生所需要的 Sprite file name int plist 
		// 此處取得的都是相對於 csbRoot 所在位置的相對座標
		// 在計算 edgeShape 的相對應座標時，必須進行轉換
		sprintf(tmp, "triangle1_%02d", i);
		auto triSprite = (Sprite *)_csbRoot->getChildByName(tmp);
		Size ts = triSprite->getContentSize();
		Point loc = triSprite->getPosition();
		float scaleX = triSprite->getScaleX();	// 對 X 軸放大
		float scaleY = triSprite->getScaleY();	// 對 Y 軸放大

		Point lep[3], wep[3];	// triShape 的三個頂點, 0 頂點、 1 左下、 2 右下
		lep[0].x = 0;  lep[0].y = (ts.height - 2) / 2.0f;
		lep[1].x = -(ts.width - 2) / 2.0f; lep[1].y = -(ts.height - 2) / 2.0f;
		lep[2].x = (ts.width - 2) / 2.0f; lep[2].y = -(ts.height - 2) / 2.0f;

		// Step1: 先CHECK 有無旋轉，有旋轉則進行端點的計算
		cocos2d::Mat4 modelMatrix, rotMatrix;
		modelMatrix.m[0] = scaleX;  // 先設定 X 軸的縮放
		modelMatrix.m[5] = scaleY;  // 先設定 Y 軸的縮放
		for (size_t j = 0; j < 3; j++)
		{   // 納入縮放與旋轉的 local space 的座標計算
			wep[j].x = lep[j].x * modelMatrix.m[0] + lep[j].y * modelMatrix.m[1];
			wep[j].y = lep[j].x * modelMatrix.m[4] + lep[j].y * modelMatrix.m[5];
		}
		b2Vec2 vecs[] = {
			b2Vec2(wep[0].x / PTM_RATIO, wep[0].y / PTM_RATIO),
			b2Vec2(wep[1].x / PTM_RATIO, wep[1].y / PTM_RATIO),
			b2Vec2(wep[2].x / PTM_RATIO, wep[2].y / PTM_RATIO)
		};
		triShape.Set(vecs, 3);
		bodyDef.type = b2_staticBody;
		bodyDef.position.Set(loc.x / PTM_RATIO, loc.y / PTM_RATIO);
		seesawBasedbody = _b2World->CreateBody(&bodyDef);
		seesawBasedbody->CreateFixture(&fixtureDef);
	}

	// 產生蹺蹺板，此為動態，每一個動態物體在 b2World 中都必須建立實體
	// 設定這個 Body 為 動態的，並在 _b2World 中建立實體，
	bodyDef.type = b2_dynamicBody;
	b2PolygonShape seesawBoardShape;
	fixtureDef.shape = &seesawBoardShape;
	fixtureDef.density = 5.0f;
	fixtureDef.friction = 0.1f;
	fixtureDef.restitution = 0.1f;
	// 取得代表蹺蹺板的圖示，並設定成 sprite
	auto boardSprite = (Sprite *)_csbRoot->getChildByName("seesawBoard");
	bodyDef.userData = boardSprite;
	Size ts = boardSprite->getContentSize();
	Point loc = boardSprite->getPosition();
	float scaleX = boardSprite->getScaleX();	// 對矩形圖示 X 軸縮放值
	float scaleY = boardSprite->getScaleY();	// 對矩形圖示 Y 軸縮放值

												// 設定板子所在的位置，因為是使用 joint 可以不用設定位置
												//	bodyDef.position.Set(loc.x/ PTM_RATIO, loc.y/ PTM_RATIO); 
	b2Body *seesawBoardbody = _b2World->CreateBody(&bodyDef); // 在 b2World 中建立實體

															  // 算出 seesawBoard 的縮放後的寬高, 4 為預留的寬度，不跟其他的圖片重疊
	float bw = (ts.width - 4)* scaleX;
	float bh = (ts.height - 4)* scaleY;

	// 設定剛體的範圍是一個 BOX （可以縮放成矩形）
	seesawBoardShape.SetAsBox(bw*0.5f / PTM_RATIO, bh*0.5f / PTM_RATIO);
	seesawBoardbody->CreateFixture(&fixtureDef);

	// 建立與基底三角形的 Joint 連結
	b2RevoluteJointDef seesawJoint;
	seesawJoint.bodyA = seesawBasedbody;
	seesawJoint.localAnchorA.Set(0, 1.2f);
	seesawJoint.bodyB = seesawBoardbody;
	seesawJoint.localAnchorB.Set(0, 0);
	_b2World->CreateJoint(&seesawJoint);

	// 讀取並建立兩顆球的實體
	// ball1 with density=10
	bodyDef.type = b2_dynamicBody;
	b2CircleShape circleShape;
	fixtureDef.shape = &circleShape;
	fixtureDef.density = 2.0f;
	fixtureDef.friction = 0.2f;
	fixtureDef.restitution = 0.15f;

	auto ballSprite = (Sprite *)_csbRoot->getChildByName("densityball1");
	bodyDef.userData = ballSprite; // body 與 Sprite 連結

	Point ballloc = ballSprite->getPosition();
	Size ballsize = ballSprite->getContentSize();
	float ballScale = ballSprite->getScale();
	// 根據 Sprite 所在位置設定圓形在 b2World 中的位置
	bodyDef.position.Set(ballloc.x / PTM_RATIO, ballloc.y / PTM_RATIO);
	b2Body* ball1 = _b2World->CreateBody(&bodyDef);
	// 設定圓形的半徑
	circleShape.m_radius = ballsize.width *ballScale * 0.5f / PTM_RATIO;
	ball1->CreateFixture(&fixtureDef);

	// ball1 with density=10000
	fixtureDef.density = 0.5f;
	ballSprite = (Sprite *)_csbRoot->getChildByName("densityball2");
	bodyDef.userData = ballSprite;
	ballloc = ballSprite->getPosition();
	ballsize = ballSprite->getContentSize();
	ballScale = ballSprite->getScale();
	bodyDef.position.Set(ballloc.x / PTM_RATIO, ballloc.y / PTM_RATIO);
	b2Body* ball2 = _b2World->CreateBody(&bodyDef);
	circleShape.m_radius = ballsize.width *ballScale * 0.5f / PTM_RATIO;
	ball2->CreateFixture(&fixtureDef);

	// ball1 with density=10000
	fixtureDef.density = 0.5f;
	ballSprite = (Sprite *)_csbRoot->getChildByName("densityball3");
	bodyDef.userData = ballSprite;
	ballloc = ballSprite->getPosition();
	ballsize = ballSprite->getContentSize();
	ballScale = ballSprite->getScale();
	bodyDef.position.Set(ballloc.x / PTM_RATIO, ballloc.y / PTM_RATIO);
	ball2 = _b2World->CreateBody(&bodyDef);
	circleShape.m_radius = ballsize.width *ballScale * 0.5f / PTM_RATIO;
	ball2->CreateFixture(&fixtureDef);

	for (int i = 4; i <= 10; i++)
	{
		sprintf(tmp, "densityball%d", i);
		// ball1 with density=10000
		fixtureDef.density = 0.5f;
		ballSprite = (Sprite *)_csbRoot->getChildByName(tmp);
		bodyDef.userData = ballSprite;
		ballloc = ballSprite->getPosition();
		ballsize = ballSprite->getContentSize();
		ballScale = ballSprite->getScale();
		bodyDef.position.Set(ballloc.x / PTM_RATIO, ballloc.y / PTM_RATIO);
		ball2 = _b2World->CreateBody(&bodyDef);
		circleShape.m_radius = ballsize.width *ballScale * 0.5f / PTM_RATIO;
		ball2->CreateFixture(&fixtureDef);
	}
}
void JointScene::setupSensor()
{

	// 以場景中所放置的兩張圖示，名稱分別是 sensor1 與 sensor2
	// 在兩個 sensor 的位置建立靜態物體，不進行碰撞，但使用碰撞事件來感應球是否經過
	// 當球往下經過 sensor1 時，累計加速的次數
	// 直到球離開感應區時，給一個反向力，讓其往上彈出
	// 設定一個變數，讓球是反向彈出時，忽略經過  sensor1 的碰撞事件
	// 當球只要經過 sensor2，就讓 _bReleasingBall 為true 代表可以釋放下一個球

	// 設定兩個靜態物體當成感應器
	char tmp[20] = "";
	for (int i = 1; i <= 2; i++)
	{
		sprintf(tmp, "sensor%02d", i);
		auto sensorSprite = (Sprite *)_csbRoot->getChildByName(tmp);
		Point loc = sensorSprite->getPosition();
		Size  size = sensorSprite->getContentSize();
		float scale = sensorSprite->getScale();
		sensorSprite->setVisible(false);
		b2BodyDef sensorBodyDef;
		sensorBodyDef.position.Set(loc.x / PTM_RATIO, loc.y / PTM_RATIO);
		sensorBodyDef.type = b2_staticBody;

		b2Body* SensorBody = _b2World->CreateBody(&sensorBodyDef);
		b2PolygonShape sensorShape;
		sensorShape.SetAsBox(size.width *0.5f * scale / PTM_RATIO, size.height*0.5f*scale / PTM_RATIO);

		b2FixtureDef SensorFixtureDef;
		SensorFixtureDef.shape = &sensorShape;
		SensorFixtureDef.isSensor = true;	// 設定為 Sensor
		SensorFixtureDef.density = 9999 + i; // 故意設定成這個值，方便碰觸時候的判斷
		SensorBody->CreateFixture(&SensorFixtureDef);
	}
}
void JointScene::readBlocksCSBFile(const char *csbfilename)
{
	auto csbRoot = CSLoader::createNode(csbfilename);
	csbRoot->setPosition(_visibleSize.width / 2.0f, _visibleSize.height / 2.0f);
	addChild(csbRoot, 1);
	char tmp[20] = "";
	for (size_t i = 1; i <= 3; i++)
	{
		// 產生所需要的 Sprite file name int plist 
		sprintf(tmp, "block1_%02d", i); 
	}
}

void JointScene::readSceneFile(const char *csbfilename)
{
	auto csbRoot = CSLoader::createNode(csbfilename);
	csbRoot->setPosition(_visibleSize.width / 2.0f, _visibleSize.height / 2.0f);
	addChild(csbRoot, 1);
	char tmp[20] = "";
	for (size_t i = 1; i <= 3; i++)
	{
		// 產生所需要的 Sprite file name int plist 
		sprintf(tmp, "XXX_%02d", i);
	}
}

void JointScene::doStep(float dt)
{
	int velocityIterations = 8;	// 速度迭代次數
	int positionIterations = 1; // 位置迭代次數 迭代次數一般設定為8~10 越高越真實但效率越差
	// Instruct the world to perform a single step of simulation.
	// It is generally best to keep the time step and iterations fixed.
	_b2World->Step(dt, velocityIterations, positionIterations);

	// 取得 _b2World 中所有的 body 進行處理
	// 最主要是根據目前運算的結果，更新附屬在 body 中 sprite 的位置
	for (b2Body* body = _b2World->GetBodyList(); body; body = body->GetNext())
	{
		// 以下是以 Body 有包含 Sprite 顯示為例
		if (body->GetUserData() != NULL)
		{
			Sprite *ballData = (Sprite*)body->GetUserData();
			ballData->setPosition(body->GetPosition().x*PTM_RATIO, body->GetPosition().y*PTM_RATIO);
			ballData->setRotation(-1 * CC_RADIANS_TO_DEGREES(body->GetAngle()));
		}
	}

	// 產生火花
	if (_contactListener._bCreateSpark) {
		_contactListener._bCreateSpark = false;	//產生完關閉
												// 判斷延遲的時間是否滿足
		if (_bSparking) { //可以噴發，實現這次的噴發
			_tdelayTime = 0; // 時間重新設定，
			_bSparking = false; // 開始計時
			for (int i = 0; i < _contactListener._NumOfSparks; i++) {
				// 建立 Spark Sprite 並與目前的物體結合
				auto sparkSprite = Sprite::createWithSpriteFrameName("bubble.png");
				sparkSprite->setColor(Color3B(rand() % 256, rand() % 256, rand() % 156));
				sparkSprite->setBlendFunc(BlendFunc::ADDITIVE);
				this->addChild(sparkSprite, 5);
				//產生小方塊資料
				b2BodyDef RectBodyDef;
				RectBodyDef.position.Set(_contactListener._createLoc.x, _contactListener._createLoc.y);
				RectBodyDef.type = b2_dynamicBody;
				RectBodyDef.userData = sparkSprite;
				b2PolygonShape RectShape;
				RectShape.SetAsBox(5 / PTM_RATIO, 5 / PTM_RATIO);
				b2Body* RectBody = _b2World->CreateBody(&RectBodyDef);
				b2FixtureDef RectFixtureDef;
				RectFixtureDef.shape = &RectShape;
				RectFixtureDef.density = 1.0f;
				RectFixtureDef.isSensor = true;
				b2Fixture*RectFixture = RectBody->CreateFixture(&RectFixtureDef);

				//給力量
				RectBody->ApplyForce(b2Vec2(rand() % 51 - 25, 50 + rand() % 30), _contactListener._createLoc, true);
			}
		}
	}
	if (!_bSparking) {
		_tdelayTime += dt;
		if (_tdelayTime >= 0.075f) {
			_tdelayTime = 0; // 歸零
			_bSparking = true; // 可進行下一次的噴發
		}
	}
	//產生煙霧
	_SmokeLoc = _carBodybody->GetWorldCenter() + b2Vec2(0, -30 / PTM_RATIO);//更新車子位置
	if (_carBodybody->GetWorldCenter().y <= 100 / PTM_RATIO) { _bStartSmoke = true; fail += 1; }
	else _bStartSmoke = false;
	//============車子的煙霧===============
	if (_bStartSmoke) {
		_bStartSmoke = false;	//產生完關閉
		// 判斷延遲的時間是否滿足
		if (_b02Smoke) { //可以噴發，實現這次的噴發
			_t02delayTime = 0; // 時間重新設定，煙
			_b02Smoke = false; // 開始計時
			for (int i = 0; i < _contactListener._NumOfSparks; i++) {
				// 建立 Spark Sprite 並與目前的物體結合
				auto sparkSprite = Sprite::createWithSpriteFrameName("circle.png");
				//sparkSprite->setColor(Color3B(255, rand() % 256, rand() % 156));
				sparkSprite->setBlendFunc(BlendFunc::ADDITIVE);
				this->addChild(sparkSprite, 5);
				//產生小方塊資料
				b2BodyDef RectBodyDef;
				RectBodyDef.position.Set(_SmokeLoc.x, _SmokeLoc.y);
				RectBodyDef.type = b2_dynamicBody;
				RectBodyDef.userData = sparkSprite;
				b2PolygonShape RectShape;
				RectShape.SetAsBox(5 / PTM_RATIO, 5 / PTM_RATIO);
				b2Body* RectBody = _b2World->CreateBody(&RectBodyDef);
				b2FixtureDef RectFixtureDef;
				RectFixtureDef.shape = &RectShape;
				RectFixtureDef.density = 1.0f;
				RectFixtureDef.isSensor = true;
				b2Fixture*RectFixture = RectBody->CreateFixture(&RectFixtureDef);

				//給力量
				RectBody->ApplyForce(b2Vec2(rand() % 5 - 2, 5 + rand() % 30), _SmokeLoc, true);
			}
		}
	}
	if (!_b02Smoke) {
		_t02delayTime += dt;
		if (_t02delayTime >= 0.75f) {
			_t02delayTime = 0; // 歸零
			_b02Smoke = true; // 可進行下一次的噴發
		}
	}
	if (fail>=600){ _csbRoot->getChildByName("Text_Fail")->setVisible(true); }
	if (_carBodybody->GetWorldCenter().x < 50 / PTM_RATIO&&fail<500)_csbRoot->getChildByName("Text_Success")->setVisible(true);
}

bool JointScene::onTouchBegan(cocos2d::Touch *pTouch, cocos2d::Event *pEvent)//觸碰開始事件
{
	Point touchLoc = pTouch->getLocation();

	// For Mouse Joiint 
	for (b2Body* body = _b2World->GetBodyList(); body; body = body->GetNext())
	{
		if (body->GetUserData() == NULL ) continue; // 靜態物體不處理
		// 判斷點的位置是否落在動態物體一定的範圍
		Sprite *spriteObj = (Sprite*)body->GetUserData();
		Size objSize = spriteObj->getContentSize();
		float fdist = MAX_2(objSize.width, objSize.height)/2.0f;
		float x = body->GetPosition().x*PTM_RATIO- touchLoc.x;
		float y = body->GetPosition().y*PTM_RATIO - touchLoc.y;
		float tpdist = x*x + y*y;
		if ( tpdist < fdist*fdist)
		{
			_bTouchOn = true;
			b2MouseJointDef mouseJointDef;
			mouseJointDef.bodyA = _bottomBody;
			mouseJointDef.bodyB = body;
			mouseJointDef.target = b2Vec2(touchLoc.x / PTM_RATIO, touchLoc.y / PTM_RATIO);
			mouseJointDef.collideConnected = true;
			mouseJointDef.maxForce = 1000.0f * body->GetMass();
			_MouseJoint = (b2MouseJoint*)_b2World->CreateJoint(&mouseJointDef); // 新增 Mouse Joint
			body->SetAwake(true);
			break;
		}
	}
	return true;
}

void  JointScene::onTouchMoved(cocos2d::Touch *pTouch, cocos2d::Event *pEvent) //觸碰移動事件
{
	Point touchLoc = pTouch->getLocation();
	if (_bTouchOn)
	{
		_MouseJoint->SetTarget(b2Vec2(touchLoc.x / PTM_RATIO, touchLoc.y / PTM_RATIO));
	}
}

void  JointScene::onTouchEnded(cocos2d::Touch *pTouch, cocos2d::Event *pEvent) //觸碰結束事件 
{
	Point touchLoc = pTouch->getLocation();
	if (_bTouchOn)
	{
		_bTouchOn = false;
		if (_MouseJoint != NULL )
		{
			_b2World->DestroyJoint(_MouseJoint);
			_MouseJoint = NULL;
		}
	}
}

void JointScene::createStaticBoundary()
{
	// 先產生 Body, 設定相關的參數

	b2BodyDef bodyDef;
	bodyDef.type = b2_staticBody; // 設定這個 Body 為 靜態的
	bodyDef.userData = NULL;
	// 在 b2World 中產生該 Body, 並傳回產生的 b2Body 物件的指標
	// 產生一次，就可以讓後面所有的 Shape 使用
	b2Body *body = _b2World->CreateBody(&bodyDef);

	_bottomBody = body;
	// 產生靜態邊界所需要的 EdgeShape
	b2EdgeShape edgeShape;
	b2FixtureDef edgeFixtureDef; // 產生 Fixture
	edgeFixtureDef.shape = &edgeShape;
	// bottom edge
	edgeShape.Set(b2Vec2(0.0f / PTM_RATIO, 0.0f / PTM_RATIO), b2Vec2(_visibleSize.width / PTM_RATIO, 0.0f / PTM_RATIO));
	body->CreateFixture(&edgeFixtureDef);

	// left edge
	edgeShape.Set(b2Vec2(0.0f / PTM_RATIO, 0.0f / PTM_RATIO), b2Vec2(0.0f / PTM_RATIO, _visibleSize.height / PTM_RATIO));
	body->CreateFixture(&edgeFixtureDef);

	// right edge
	edgeShape.Set(b2Vec2(_visibleSize.width / PTM_RATIO, 0.0f / PTM_RATIO), b2Vec2(_visibleSize.width / PTM_RATIO, _visibleSize.height / PTM_RATIO));
	body->CreateFixture(&edgeFixtureDef);

	// top edge
	edgeShape.Set(b2Vec2(0.0f / PTM_RATIO, _visibleSize.height / PTM_RATIO), b2Vec2(_visibleSize.width / PTM_RATIO, _visibleSize.height / PTM_RATIO));
	body->CreateFixture(&edgeFixtureDef);


}
//==============分子特效===============
CContactListener::CContactListener()
{
	_bApplyImpulse = false;
	_bCreateSpark = false;
	_NumOfSparks = 5;
}
void CContactListener::setCollisionTarget(cocos2d::Sprite &targetSprite)
{
	_targetSprite = &targetSprite;
}

void CContactListener::BeginContact(b2Contact* contact)
{
	b2Body* BodyA = contact->GetFixtureA()->GetBody();
	b2Body* BodyB = contact->GetFixtureB()->GetBody();

	// check 是否為落下的球經過 sensor1 ，只要經過就立刻讓他彈出去
	if (BodyA->GetFixtureList()->GetDensity() == 10000.0f) { // 代表 sensor1
		BodyB->ApplyLinearImpulse(b2Vec2(0, 50 + rand() % 101), BodyB->GetWorldCenter(), true);
		_bApplyImpulse = true;
	}
	else if (BodyB->GetFixtureList()->GetDensity() == 10000.0f) {// 代表 sensor1
		BodyA->ApplyLinearImpulse(b2Vec2(0, 50 + rand() % 101), BodyB->GetWorldCenter(), true);
		_bApplyImpulse = true;
	}

	if (BodyA->GetUserData() == _targetSprite) {
		float lengthV = BodyB->GetLinearVelocity().Length();
		if (lengthV >= 4.25f) { // 接觸時的速度超過一定的值才噴出火花
			_bCreateSpark = true;
			_createLoc = BodyA->GetWorldCenter() + b2Vec2(0, -30 / PTM_RATIO);
		}
	}
	else if (BodyB->GetUserData() == _targetSprite) {
		float lengthV = BodyB->GetLinearVelocity().Length();
		if (lengthV >= 4.25f) { // 接觸時的速度超過一定的值才噴出火花
			_bCreateSpark = true;
			_createLoc = BodyB->GetWorldCenter() + b2Vec2(0, -30 / PTM_RATIO);
		}
	}
}

//碰撞結束
void CContactListener::EndContact(b2Contact* contact)
{
	b2Body* BodyA = contact->GetFixtureA()->GetBody();
	b2Body* BodyB = contact->GetFixtureB()->GetBody();

	if (BodyA->GetFixtureList()->GetDensity() == 10001.0f && _bApplyImpulse) { // 代表 sensor2
		BodyA->GetFixtureList()->SetDensity(10002);
		_bApplyImpulse = false;
	}
	else if (BodyB->GetFixtureList()->GetDensity() == 10001.0f && _bApplyImpulse) {	// 代表 sensor2
		BodyB->GetFixtureList()->SetDensity(10002);
		_bApplyImpulse = false;
	}
}

#ifdef BOX2D_DEBUG
//改寫繪製方法
void JointScene::draw(Renderer *renderer, const Mat4 &transform, uint32_t flags)
{
	Director* director = Director::getInstance();

	GL::enableVertexAttribs(cocos2d::GL::VERTEX_ATTRIB_FLAG_POSITION);
	director->pushMatrix(MATRIX_STACK_TYPE::MATRIX_STACK_MODELVIEW);
	_b2World->DrawDebugData();
	director->popMatrix(MATRIX_STACK_TYPE::MATRIX_STACK_MODELVIEW);
}
#endif