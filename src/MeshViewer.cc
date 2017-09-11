//-----------------------------------------------------------------------------
//  MeshViewer.cc
//-----------------------------------------------------------------------------
#include "Pre.h"
#include "Core/Main.h"
#include "Core/String/StringBuilder.h"
#include "Gfx/Gfx.h"
#include "IO/IO.h"
#include "IMUI/IMUI.h"
#include "Input/Input.h"
#include "HttpFS/HTTPFileSystem.h"
#include "LocalFS/LocalFileSystem.h"
#include "Assets/Gfx/MeshLoader.h"
#include "glm/mat4x4.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtx/polar_coordinates.hpp"
#include "shaders.h"

#include "NKUI/NKUI.h"




#include <thread>
#include <chrono>

#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;
using namespace Oryol;


//////////////////////////////////////////////////////////////////////////////////////
////////////////////// 3D Rotations and Translations /////////////////////////////////

//////////////// Translations and rotations of a 3-D point (useful for the parametrized surfaces of the milling tool) /////////


glm::vec3 TRASL_X_Y_Z_3(glm::vec3 v_in1, float x, float y, float z)
{
	glm::vec3 v_out1;
	v_out1.x = v_in1.x + x;
	v_out1.y = v_in1.y + y;
	v_out1.z = v_in1.z + z;
	return v_out1;
}

glm::vec3 ROT_X_3(glm::vec3 v_in2, float theta_x)
{
	glm::vec3 v_out2;
	v_out2.x = v_in2.x;
	v_out2.y = v_in2.y*cos(theta_x) - v_in2.z*sin(theta_x);
	v_out2.z = v_in2.x*sin(theta_x) + v_in2.z*cos(theta_x);
	return v_out2;
}

glm::vec3 ROT_Y_3(glm::vec3 v_in3, float theta_y)
{
	glm::vec3 v_out3;
	v_out3.x = v_in3.x*cos(theta_y) + v_in3.z*sin(theta_y);
	v_out3.y = v_in3.y;
	v_out3.z = -v_in3.x*sin(theta_y) + v_in3.z*cos(theta_y);
	return v_out3;
}

glm::vec3 ROT_Z_3(glm::vec3 v_in4, float theta_z)
{
	glm::vec3 v_out4;
	v_out4.x = v_in4.x*cos(theta_z) - v_in4.y*sin(theta_z);
	v_out4.y = v_in4.x*sin(theta_z) + v_in4.y*cos(theta_z);
	v_out4.z = v_in4.z;
	return v_out4;
}

//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////







class MeshViewerApp : public App {
public:
	AppState::Code OnInit();
	AppState::Code OnRunning();
	AppState::Code OnCleanup();

	//ShapeBuilder shapeBuilder;

	PassAction passAction;

private:
	const float  M_PI = 3.14159265358979323846f;
	void handleInput();
	void updateCamera_initial(int num, float NC_coord_T_R[6]);
	void updateCamera(int num, float NC_coord_T_R[6]);
	void updateLight();
	void createMaterials(int num);

	void loadMesh(const char* path, int num);
	void applyVariables(int materialIndex, int num);

	void drawUI();   ///// UI Interface


					 /////////////////////// Some Config. Variables ///////////////////

	static const int numMeshes = 3;
	static const char* meshNames[numMeshes];
	static const char* meshPaths[numMeshes];
	static const int numShaders = 3;
	int frameCount = 0;
	//////////////////////////////////////////////////////////////////

	glm::vec3 eyePos;
	glm::mat4 view[numMeshes];
	glm::mat4 proj;
	glm::mat4 model[numMeshes];
	glm::mat4 modelViewProj[numMeshes];

	Id mesh[numMeshes];
	Id mesh2;

	MeshSetup curMeshSetup[numMeshes];
	MeshSetup curMeshSetup2;

	ResourceLabel curMeshLabel[numMeshes];
	ResourceLabel curMeshLabel1;
	ResourceLabel curMeshLabel2;


	/////////////// NC-Coordinates ///////////////////

	float NC_coord[6][numMeshes];
	glm::vec3 poi[numMeshes];

	//////////////////////////////////////////////////

	ifstream myfile_NC_coord;


	//////////////////////////////////////////////////





	static const char* shaderNames[numShaders];
	enum {
		Normals = 0,
		Lambert,
		Phong
	};
	Id shaders[numShaders];


	ResourceLabel curMaterialLabel[numMeshes];


	int numMaterials[numMeshes];
	int numMaterials2 = 0;


	struct Material {
		int shaderIndex = Phong;
		Id pipeline;
		glm::vec4 diffuse = glm::vec4(0.0f, 0.24f, 0.64f, 1.0f);
		glm::vec4 specular = glm::vec4(1.0f, 1.0f, 1.0f, 1.0f);
		float specPower = 32.0f;
	} materials[GfxConfig::MaxNumPrimGroups][numMeshes], materials2[GfxConfig::MaxNumPrimGroups];
	bool gammaCorrect = true;



	struct CameraSetting {
		float dist = 50.0f;
		glm::vec2 orbital = glm::vec2(glm::radians(10.0f), 0.0f);
		float height = 10.0f;
		float height2 = 0.0f;
		glm::vec2 startOrbital;
		float startDistance = 0.0f;
	} camera;
	bool camAutoOrbit = true;
	struct CameraSetting cameraSettings[numMeshes];



	glm::vec2 lightOrbital = glm::vec2(glm::radians(25.0f), 0.0f);
	glm::vec3 lightDir;
	glm::vec4 lightColor = glm::vec4(1.0f, 1.0f, 1.0f, 1.0f);
	float lightIntensity = 1.0f;
	bool lightAutoOrbit = false;
	bool dragging = false;

	StringBuilder strBuilder;

	const float minCamDist = -80.0f;
	const float maxCamDist = 80.0f;
	const float minLatitude = -85.0f;
	const float maxLatitude = 85.0f;
	const float minCamHeight = -10.0f;
	const float maxCamHeight = 10.0f;

	bool load_status = false;
	bool anim_status = false;
	bool pause_status = false;


};
OryolMain(MeshViewerApp);



const char* MeshViewerApp::meshNames[numMeshes] = {
	"Tiger",
	"Blitz",
	"Teapot"
};
const char* MeshViewerApp::meshPaths[numMeshes] = {
	//"data:Shapertool.omsh",
	//"data:Shapertool_1.omsh",
	"data:Tiger.omsh.txt",
	"data:opelblitz.omsh.txt",
	"data:teapot.omsh.txt"
};

const char* MeshViewerApp::shaderNames[numShaders] = {
	"Normals",
	"Lambert",
	"Phong"
};



////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
//////////////////////// INITIALIZATION OF MESHVIEWER CLASS ///////////////////////////////

AppState::Code
MeshViewerApp::OnInit() {

	// setup IO system
	IOSetup ioSetup;

	ioSetup.FileSystems.Add("file", LocalFileSystem::Creator());
	ioSetup.Assigns.Add("data:", "file:///C:/Users/ivand/Documents/UI_WASS/oryol-samples/data/");

	IO::Setup(ioSetup);

	////////////////////////////////// WINDOWS SETUP  ///////////////////////////////////////

	auto gfxSetup = GfxSetup::WindowMSAA4(800, 512, "EVOMECS COLLISION APP");

	gfxSetup.HighDPI = true;
	gfxSetup.DefaultPassAction = PassAction::Clear(glm::vec4(0.941176f, 0.972549f, 1.0f, 1.0f));


	Gfx::Setup(gfxSetup);

	///////////////////// MOUSE SETUP (https://developer.mozilla.org/en-US/docs/Web/API/Pointer_Lock_API)  ///////////

	Input::Setup();
	Input::SetPointerLockHandler([this](const InputEvent& event) -> PointerLockMode::Code {
		if (event.Button == MouseButton::Left) {
			if (event.Type == InputEvent::MouseButtonDown) {
				if (!ImGui::IsMouseHoveringAnyWindow()) {
					this->dragging = true;
					return PointerLockMode::Enable;
				}
			}
			else if (event.Type == InputEvent::MouseButtonUp) {
				if (this->dragging) {
					this->dragging = false;
					return PointerLockMode::Disable;
				}
			}
		}
		return PointerLockMode::DontCare;
	});



	////////////////////////////////////////////////////////////////////////
	/////////////////////// Shaders Definitions ////////////////////////////

	this->shaders[Normals] = Gfx::CreateResource(NormalsShader::Setup());
	this->shaders[Lambert] = Gfx::CreateResource(LambertShader::Setup());
	this->shaders[Phong] = Gfx::CreateResource(PhongShader::Setup());



	/////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////


	// setup projection and view matrices
	const float fbWidth = (const float)Gfx::DisplayAttrs().FramebufferWidth;
	const float fbHeight = (const float)Gfx::DisplayAttrs().FramebufferHeight;
	this->proj = glm::perspectiveFov(glm::radians(60.0f), fbWidth, fbHeight, 0.01f, 100.0f);



	///////// Setup NC_coordinates for all Meshes ////////////

	///////// External File /////////
	//	myfile_NC_coord.open("C:/Users/ivand/Desktop/GPU_BVH_Coll/GPU_BVH_Coll/c++_stl_files/NC_coord_transl_rot.txt");

	//// Dynamical generated ///////

	for (int i = 0; i < numMeshes; ++i)
		for (int j = 0; j < 6; ++j)
			NC_coord[j][i] = 0.0f;


	/////// Mesh1 ///////
	NC_coord[0][0] = 0.0f;
	NC_coord[1][0] = 0.0f;
	NC_coord[2][0] = 0.0f;
	NC_coord[3][0] = 0.0f;
	NC_coord[4][0] = 0.0f;
	NC_coord[5][0] = 0.0f;//1.0f * this->M_PI/6.0f;

	/////// Mesh2 ///////
	NC_coord[0][1] = 20.0f;
	NC_coord[1][1] = 5.0f;
	NC_coord[2][1] = 4.0f;
	NC_coord[3][1] = 0.0f;
	NC_coord[4][1] = 0.0f;
	NC_coord[5][1] = 0.0f;//2.0f *  this->M_PI / 6.0f;
						  
	/////// Mesh3 ///////
	NC_coord[0][2] = -10.0f;
	NC_coord[1][2] = -10.0f;
	NC_coord[2][2] = 0.0f;
	NC_coord[3][2] = 0.0f;
	NC_coord[4][2] = 0.0f;
	NC_coord[5][2] = 0.0f;//3.0f *  this->M_PI / 6.0f;


	for (int i = 0; i < numMeshes; ++i)
		poi[i] = glm::vec3(0.0f, this->camera.height, 0.0f);

	//////////////////////////////////////////////////////////




	/////////////////////////////////////////////////////////////////
	///////////////////////////// UI SETUP //////////////////////////

	// setup IMUI ui system
	IMUI::Setup();
	ImGuiStyle& style = ImGui::GetStyle();
	style.WindowRounding = 0.0f;
	style.TouchExtraPadding.x = 5.0f;
	style.TouchExtraPadding.y = 5.0f;
	ImVec4 defaultBlue(0.0f, 0.5f, 1.0f, 0.7f);
	style.Colors[ImGuiCol_TitleBg] = defaultBlue;
	style.Colors[ImGuiCol_TitleBgCollapsed] = defaultBlue;
	style.Colors[ImGuiCol_SliderGrab] = defaultBlue;
	style.Colors[ImGuiCol_SliderGrabActive] = defaultBlue;
	style.Colors[ImGuiCol_Button] = defaultBlue;
	style.Colors[ImGuiCol_ButtonHovered] = ImVec4(0.0f, 0.5f, 1.0f, 0.3f);
	style.Colors[ImGuiCol_ButtonActive] = ImVec4(0.0f, 0.5f, 1.0f, 1.0f);
	style.Colors[ImGuiCol_ScrollbarGrab] = ImVec4(0.0f, 0.5f, 1.0f, 0.5f);
	style.Colors[ImGuiCol_ScrollbarGrabActive] = ImVec4(0.0f, 0.5f, 1.0f, 1.0f);
	style.Colors[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.0f, 0.5f, 1.0f, 0.3f);
	style.Colors[ImGuiCol_Header] = defaultBlue;
	style.Colors[ImGuiCol_HeaderHovered] = defaultBlue;
	style.Colors[ImGuiCol_HeaderActive] = defaultBlue;


	ImGuiIO& IO = ImGui::GetIO();
	IO.FontGlobalScale = 1.6f;




	return App::OnInit();
}





void MeshViewerApp::loadMesh(const char* path, int num) {

	// unload current mesh
	if (this->curMeshLabel[num].IsValid()) {
		Gfx::DestroyResources(this->curMeshLabel[num]);
		this->curMeshSetup[num] = MeshSetup();
	}

	// load new mesh, use 'onloaded' callback to capture the mesh setup
	// object of the loaded mesh
	this->numMaterials[num] = 0;
	this->curMeshLabel[num] = Gfx::PushResourceLabel();
	this->mesh[num] = Gfx::LoadResource(MeshLoader::Create(MeshSetup::FromFile(path), [this, num](MeshSetup& setup) {
		this->curMeshSetup[num] = setup;
		this->numMaterials[num] = setup.NumPrimitiveGroups();
		this->createMaterials(num);
	}));
	Gfx::PopResourceLabel();
}



void MeshViewerApp::createMaterials(int num) {


	o_assert_dbg(this->mesh[num].IsValid());
	if (this->curMaterialLabel[num].IsValid()) {
		Gfx::DestroyResources(this->curMaterialLabel[num]);
	}

	this->curMaterialLabel[num] = Gfx::PushResourceLabel();



	for (int i = 0; i < this->numMaterials[num]; i++) {
		auto ps = PipelineSetup::FromLayoutAndShader(this->curMeshSetup[num].Layout, this->shaders[this->materials[i][num].shaderIndex]);
		ps.DepthStencilState.DepthWriteEnabled = true;
		ps.DepthStencilState.DepthCmpFunc = CompareFunc::LessEqual;
		ps.RasterizerState.CullFaceEnabled = true;
		ps.RasterizerState.SampleCount = 4;
		this->materials[i][num].pipeline = Gfx::CreateResource(ps);
	}
	Gfx::PopResourceLabel();
}








//////////////////////////////////////////////////////////////////////////////////
///////////////////////////// ON RUNNING /////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////// ONRUNNING MESH VIEWER /////////////////////////////////////////


//-----------------------------------------------------------------------------
AppState::Code
MeshViewerApp::OnRunning() {

	this->frameCount++;
	this->handleInput();



	if (this->load_status == true)
		for (int i = 0; i < numMeshes; ++i)
		{
			float coord_initial[6] = { NC_coord[0][i], NC_coord[1][i] , NC_coord[2][i] , NC_coord[3][i] , NC_coord[4][i] , NC_coord[5][i] };
			this->updateCamera_initial(i, coord_initial);
		}

	if (this->load_status == true && this->anim_status == true && this->pause_status == false)
	{
		for (int i = 0; i < numMeshes; ++i)
		{
		//int i = 1;  //// Only update second surface
		float coord[6] = { NC_coord[0][i], NC_coord[1][i] , NC_coord[2][i] , NC_coord[3][i] , NC_coord[4][i] , NC_coord[5][i] };
		this->updateCamera(i, coord);
		}
	}



	//	nk_context* ctx = NKUI::NewFrame();
	//	if (nk_tree_push(ctx, NK_TREE_NODE, "Button", NK_MINIMIZED))
	//	{
	//	}



	this->updateLight();

	Gfx::BeginPass();

	this->drawUI();

	DrawState drawState;

	for (int j = 0; j < numMeshes; ++j)
	{
		drawState.Mesh[0] = this->mesh[j];

		for (int i = 0; i < this->numMaterials[j]; i++)
		{
			drawState.Pipeline = this->materials[i][j].pipeline;
			Gfx::ApplyDrawState(drawState);
			this->applyVariables(i, j);
			Gfx::Draw(i);
		}
	}

	ImGui::Render();

	Gfx::EndPass();

	Gfx::CommitFrame();

	return Gfx::QuitRequested() ? AppState::Cleanup : AppState::Running;

}






void MeshViewerApp::updateCamera(int num, float NC_coord_T_R[6]) {

	//////////////////////  Camera on the run //////////////////////////
	this->eyePos = glm::euclidean(this->camera.orbital) * (this->camera.dist);    //// Object trajectory with respect to World-Coord.


																				  ///////////////////// Change Mesh position ////////////////

	glm::vec3 pos_in = this->poi[num];
	glm::vec3 pos_out;


	pos_out = TRASL_X_Y_Z_3(pos_in, NC_coord_T_R[0], NC_coord_T_R[1], NC_coord_T_R[2]);
	pos_in = ROT_X_3(pos_out, NC_coord_T_R[3]);
	pos_out = ROT_Y_3(pos_in, NC_coord_T_R[4]);
	pos_in = ROT_Z_3(pos_out, NC_coord_T_R[5]);

	this->NC_coord[5][num] += this->M_PI / 180.0f;




	this->view[num] = glm::lookAt(this->eyePos + pos_in, pos_in, glm::vec3(0.0f, 1.0f, 0.0f));
	this->modelViewProj[num] = this->proj * this->view[num];
	//////////////////////////////////////////////////////////////////////
}



void MeshViewerApp::updateCamera_initial(int num, float NC_coord_T_R[6]) {

	//////////////////////  Camera on the run //////////////////////////
	this->eyePos = glm::euclidean(this->camera.orbital) * (this->camera.dist);    //// Object trajectory with respect to World-Coord.


																				  ///////////////////// Change Mesh position ////////////////

	glm::vec3 pos_in = this->poi[num];
	glm::vec3 pos_out;


	pos_out = TRASL_X_Y_Z_3(pos_in, NC_coord_T_R[0], NC_coord_T_R[1], NC_coord_T_R[2]);
	pos_in = ROT_X_3(pos_out, NC_coord_T_R[3]);
	pos_out = ROT_Y_3(pos_in, NC_coord_T_R[4]);
	pos_in = ROT_Z_3(pos_out, NC_coord_T_R[5]);



	this->view[num] = glm::lookAt(this->eyePos + pos_in, pos_in, glm::vec3(0.0f, 1.0f, 0.0f));
	this->modelViewProj[num] = this->proj * this->view[num];
	//////////////////////////////////////////////////////////////////////
}







void MeshViewerApp::applyVariables(int matIndex, int num)
{
	switch (this->materials[matIndex][num].shaderIndex) {
	case Normals:
		// Normals shader
	{
		NormalsShader::vsParams vsParams;
		vsParams.mvp = this->modelViewProj[num];
		Gfx::ApplyUniformBlock(vsParams);
	}
	break;
	case Lambert:
		// Lambert shader
	{
		LambertShader::vsParams vsParams;
		vsParams.mvp = this->modelViewProj[num];
		vsParams.model = this->model[num];
		Gfx::ApplyUniformBlock(vsParams);

		LambertShader::fsParams fsParams;
		fsParams.lightColor = this->lightColor * this->lightIntensity;
		fsParams.lightDir = this->lightDir;
		fsParams.matDiffuse = this->materials[matIndex][num].diffuse;
		fsParams.gammaCorrect = this->gammaCorrect ? 1.0f : 0.0f;
		Gfx::ApplyUniformBlock(fsParams);
	}
	break;
	case Phong:
		// Phong shader
	{
		PhongShader::vsParams vsParams;
		vsParams.mvp = this->modelViewProj[num];
		vsParams.model = this->model[num];
		Gfx::ApplyUniformBlock(vsParams);

		PhongShader::fsParams fsParams;
		fsParams.eyePos = this->eyePos;
		fsParams.lightColor = this->lightColor * this->lightIntensity;
		fsParams.lightDir = this->lightDir;
		fsParams.matDiffuse = this->materials[matIndex][num].diffuse;
		fsParams.matSpecular = this->materials[matIndex][num].specular;
		fsParams.matSpecularPower = this->materials[matIndex][num].specPower;
		fsParams.gammaCorrect = this->gammaCorrect ? 1.0f : 0.0f;
		Gfx::ApplyUniformBlock(fsParams);
	}
	break;

	default:
		o_error("Unknown shader index, FIXME!");
		break;
	}
}



///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////












///////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////// Not important functions ///////////////////////////////


void MeshViewerApp::handleInput() {

	// rotate camera with mouse if not UI-dragging
	if (Input::TouchpadAttached()) {
		if (!ImGui::IsMouseHoveringAnyWindow()) {
			if (Input::TouchPanningStarted()) {
				this->camera.startOrbital = this->camera.orbital;
			}
			if (Input::TouchPanning()) {
				glm::vec2 diff = (Input::TouchPosition(0) - Input::TouchStartPosition(0)) * 0.01f;
				this->camera.orbital.y = this->camera.startOrbital.y - diff.x;
				this->camera.orbital.x = glm::clamp(this->camera.startOrbital.x + diff.y, glm::radians(minLatitude), glm::radians(maxLatitude));
			}
			if (Input::TouchPinchingStarted()) {
				this->camera.startDistance = this->camera.dist;
			}
			if (Input::TouchPinching()) {
				float startDist = glm::length(glm::vec2(Input::TouchStartPosition(1) - Input::TouchStartPosition(0)));
				float curDist = glm::length(glm::vec2(Input::TouchPosition(1) - Input::TouchPosition(0)));
				this->camera.dist = glm::clamp(this->camera.startDistance - (curDist - startDist) * 0.01f, minCamDist, maxCamDist);
			}
		}
	}
	if (Input::MouseAttached()) {
		if (this->dragging) {
			if (Input::MouseButtonPressed(MouseButton::Left)) {
				this->camera.orbital.y -= Input::MouseMovement().x * 0.01f;
				this->camera.orbital.x = glm::clamp(
					this->camera.orbital.x + Input::MouseMovement().y * 0.01f,
					glm::radians(minLatitude),
					glm::radians(maxLatitude));
			}
		}
		this->camera.dist = glm::clamp(this->camera.dist + Input::MouseScroll().y * 0.1f, minCamDist, maxCamDist);
	}
}



void MeshViewerApp::updateLight() {
	if (this->lightAutoOrbit) {
		this->lightOrbital.y += 0.01f;
		if (this->lightOrbital.y > glm::radians(360.0f)) {
			this->lightOrbital.y = 0.0f;
		}
	}
	this->lightDir = glm::euclidean(this->lightOrbital);
}






///////////////////////////////////////////////////////////////////////
/////////////////////////////// CLEAN UP //////////////////////////////

AppState::Code
MeshViewerApp::OnCleanup() {
	IMUI::Discard();
	Input::Discard();
	Gfx::Discard();
	IO::Discard();
	return App::OnCleanup();
}













//////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////// UI GRAPHIC INTERPHASE //////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

void MeshViewerApp::drawUI() {


	IMUI::NewFrame();

	ImGui::Begin("Collision Test", nullptr, ImVec2(240, 300), 0.25f, 0);

	ImGui::PushItemWidth(130.0f);


	//////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////// LOAD SURFACE //////////////////////////////////////


	if (ImGui::Button("Load Surfaces"))
	{
		if (this->load_status == false)
		{

			for (int i = 0; i < numMeshes; ++i)
				this->loadMesh(this->meshPaths[i], i);


			for (int i = 0; i < numMeshes; ++i)
			{
				float coord_initial[6] = { NC_coord[0][i], NC_coord[1][i] , NC_coord[2][i] , NC_coord[3][i] , NC_coord[4][i] , NC_coord[5][i] };
				this->updateCamera_initial(i, coord_initial);
			}
			this->load_status = true;
		}
	}



	/////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////// COLLISION SIMULATION ///////////////////////////////////


	if (this->load_status == true)
	{

		ImGui::Text(" ");
		if (ImGui::CollapsingHeader("Simulation"))
		{
			if (ImGui::Button("PLAY"))
			{
				this->anim_status = true;
				this->pause_status = false;
			}
			if (ImGui::Button("PAUSE"))
			{
				this->pause_status = true;
			}
		}
	}




	ImGui::PopItemWidth();
	ImGui::End();






	/*

	const char* state;
	switch (Gfx::QueryResourceInfo(this->mesh).State) {
	case ResourceState::Valid: state = "Loaded"; break;
	case ResourceState::Failed: state = "Load Failed"; break;
	case ResourceState::Pending: state = "Loading..."; break;
	default: state = "Invalid"; break;
	}


	IMUI::NewFrame();
	ImGui::Begin("Mesh Viewer", nullptr, ImVec2(240, 300), 0.25f, 0);
	ImGui::PushItemWidth(130.0f);
	if (ImGui::Combo("##mesh", (int*) &this->curMeshIndex, this->meshNames, numMeshes)) {
	this->camera = this->cameraSettings[this->curMeshIndex];
	this->loadMesh(this->meshPaths[this->curMeshIndex]);
	}



	ImGui::Text("state: %s\n", state);
	if (this->curMeshSetup.NumPrimitiveGroups() > 0) {
	ImGui::Text("primitive groups:\n");
	for (int i = 0; i < this->curMeshSetup.NumPrimitiveGroups(); i++) {
	ImGui::Text("%d: %d triangles\n", i, this->curMeshSetup.PrimitiveGroup(i).NumElements / 3);
	}
	}
	if (ImGui::CollapsingHeader("Camera")) {
	ImGui::SliderFloat("Dist##cam", &this->camera.dist, minCamDist, maxCamDist);
	ImGui::SliderFloat("Height##cam", &this->camera.height, minCamHeight, maxCamHeight);
	ImGui::SliderAngle("Long##cam", &this->camera.orbital.y, 0.0f, 360.0f);
	ImGui::SliderAngle("Lat##cam", &this->camera.orbital.x, minLatitude, maxLatitude);
	ImGui::Checkbox("Auto Orbit##cam", &this->camAutoOrbit);
	if (ImGui::Button("Reset##cam")) {
	this->camera.dist = 8.0f;
	this->camera.height = 1.0f;
	this->camera.orbital = glm::vec2(0.0f, 0.0f);
	this->camAutoOrbit = false;
	}
	}
	if (ImGui::CollapsingHeader("Light")) {
	ImGui::SliderAngle("Long##light", &this->lightOrbital.y, 0.0f, 360.0f);
	ImGui::SliderAngle("Lat##light", &this->lightOrbital.x, minLatitude, maxLatitude);
	ImGui::ColorEdit3("Color##light", &this->lightColor.x);
	ImGui::SliderFloat("Intensity##light", &this->lightIntensity, 0.0f, 5.0f);
	ImGui::Checkbox("Auto Orbit##light", &this->lightAutoOrbit);
	ImGui::Checkbox("Gamma Correct##light", &this->gammaCorrect);
	if (ImGui::Button("Reset##light")) {
	this->lightOrbital = glm::vec2(glm::radians(25.0f), 0.0f);
	this->lightColor = glm::vec4(1.0f, 1.0f, 1.0f, 1.0f);
	this->lightIntensity = 1.0f;
	this->lightAutoOrbit = false;
	}
	}
	for (int i = 0; i < this->numMaterials; i++) {
	this->strBuilder.Format(32, "Material %d", i);
	if (ImGui::CollapsingHeader(this->strBuilder.AsCStr())) {
	this->strBuilder.Format(32, "shader##mat%d", i);
	if (ImGui::Combo(strBuilder.AsCStr(), &this->materials[i].shaderIndex, this->shaderNames, numShaders)) {
	this->createMaterials();
	}
	if ((Lambert == this->materials[i].shaderIndex) || (Phong == this->materials[i].shaderIndex)) {
	this->strBuilder.Format(32, "diffuse##%d", i);
	ImGui::ColorEdit3(this->strBuilder.AsCStr(), &this->materials[i].diffuse.x);
	}
	if (Phong == this->materials[i].shaderIndex) {
	this->strBuilder.Format(32, "specular##%d", i);
	ImGui::ColorEdit3(this->strBuilder.AsCStr(), &this->materials[i].specular.x);
	this->strBuilder.Format(32, "power##%d", i);
	ImGui::SliderFloat(this->strBuilder.AsCStr(), &this->materials[i].specPower, 1.0f, 512.0f);
	}
	}
	}
	ImGui::PopItemWidth();
	ImGui::End();


	*/
}