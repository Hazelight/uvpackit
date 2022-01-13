#include <string>
#include <array>
#include <unordered_map>

#include <thread>
#include <future>
#include <chrono>

// UV Packmaster
#include <uvpCore.hpp> 

#include <lxu_command.hpp>

// Includes to support Monitor ( progressbar )
#include <lx_io.hpp>
#include <lx_stddialog.hpp>

// Modo Math utilities,
#include <lxvmath.h>
#include <lxu_matrix.hpp>
#include <lxu_vector.hpp>
#include <lxu_math.hpp>

// Included to support logging to the Event Log
#include <lx_log.hpp>
#include <lxu_log.hpp>

// Modo SDK for performing layerscan,
#include <lx_layer.hpp>
#include <lx_mesh.hpp>

using namespace uvpcore;
using namespace lx_err; // gives us check()

// UV Packmaster Related classes, slightly tweaked from their FBX example,
// url: https://uvpackmaster.com/sdkdoc/90-sample-application/

typedef std::array<UvpMessageT*, static_cast<int>(UvpMessageT::MESSAGE_CODE::VALUE_COUNT)> UvpMessageArrayT;
void opExecutorMessageHandler(void* m_pMessageHandlerData, UvpMessageT* pMsg);

// Wrapper class simplifying execution of UVP operations.
class UvpOpExecutorT
{
private:
	friend void opExecutorMessageHandler(void* m_pMessageHandlerData, UvpMessageT* pMsg);

	std::list<UvpMessageT*> m_ReceivedMessages;
	UvpMessageArrayT m_LastMessagePerCode;

	bool m_DebugMode;

	UvpOperationT* operation;

	void destroyMessages()
	{
		// The application becomes the owner of UVP messages after receiving it,
		// so we have to make sure they are eventually deallocated by calling
		// the destory method on them (do not use the delete operator).
		for (UvpMessageT* pMsg : m_ReceivedMessages)
		{
			pMsg->destroy();
		}
		m_ReceivedMessages.clear();
	}

	void reset()
	{
		destroyMessages();
		m_LastMessagePerCode = { nullptr };
	}

	// This method is called every time the packer sends a message to the application.
	// We need to handle the message properly.
	// https://uvpackmaster.com/sdkdoc/20-communication-with-the-packer/
	void handleMessage(UvpMessageT* pMsg)
	{
		if (pMsg->m_Code == UvpMessageT::MESSAGE_CODE::PROGRESS_REPORT)
		{
			UvpProgressReportMessageT* pReportProgressMsg = static_cast<UvpProgressReportMessageT*>(pMsg);

			for (int i = 0; i < pReportProgressMsg->m_ProgressSize; i++)
			{
				// m_ProgressArray, An array which stores actual progress information. 
				// It contains m_ProgressSize integers ranging from 0 to 100 (percent).
				// Get the progress % from the message,
				unsigned progress = pReportProgressMsg->m_ProgressArray[i];

				// Get a local variable for the atomic uints, to shield
				// against us overwriting the progress with a lower value
				// causing the monitor to loop forever.
				unsigned current = 0;

				// Set the public facing progress so main thread can 
				// update the progress bar.
				switch (pReportProgressMsg->m_PackingPhase)
				{
				case uvpcore::UVP_PACKING_PHASE_CODE::TOPOLOGY_ANALYSIS:
					current = topology_progress;
					topology_progress = (progress > current ? progress : current);
					break;
				// PACKING and PIXEL_MARGIN_ADJUSTMENT will both push our progress bar
				// seeing how PACKING will run when using `margin` and PIXEL_MARGIN_ADJUSTMENT
				// will enter if users specify the pixels for margin/padding
				case uvpcore::UVP_PACKING_PHASE_CODE::PACKING:
				case uvpcore::UVP_PACKING_PHASE_CODE::PIXEL_MARGIN_ADJUSTMENT:
					current = packing_progress;
					packing_progress = (progress > current ? progress : current);
					break;
				default:
					break;
				}
			}
		}
		m_LastMessagePerCode[static_cast<int>(pMsg->m_Code)] = pMsg;
		m_ReceivedMessages.push_back(pMsg);
	}

public:
	// Thread safe uints to track progress of different phases,
	std::atomic_uint topology_progress = 0;
	std::atomic_uint packing_progress = 0;

	UvpOpExecutorT(bool debugMode) :
		m_DebugMode(debugMode)
	{}

	~UvpOpExecutorT()
	{
		destroyMessages();
		if (operation != nullptr)
			delete operation;
	}

	UVP_ERRORCODE execute(UvpOperationInputT& uvpInput)
	{
		reset();

		uvpInput.m_pMessageHandler = opExecutorMessageHandler;
		uvpInput.m_pMessageHandlerData = this;

		if (m_DebugMode)
		{
			// Check whether the application configurated the operation input properly.
			// WARNING: this operation is time consuming (in particular it iterates over all UV data),
			// that is why it should only be executed when debugging the application. It should
			// never be used in production.
			const char* pValidationResult = uvpInput.validate();

			if (pValidationResult)
			{
				throw std::runtime_error("Operation input validation failed: " + std::string(pValidationResult));
			}
		}

		operation = new UvpOperationT(uvpInput);

		// Start actual execution of the operation. This method won’t return
		// until the operation is done, so it must be called from a different
		// thread, if you don’t want your application to be blocked.
		// https://uvpackmaster.com/sdkdoc/10-classes/10-uvpoperationt/#ID_entry
		UVP_ERRORCODE retCode = operation->entry();

		// Being done, to ensure we don't get stuck with the monitor let's set
		// all progress to 100,
		topology_progress = 100;
		packing_progress = 100;

		return retCode;
	}

	UvpMessageT* getLastMessage(UvpMessageT::MESSAGE_CODE code)
	{
		return m_LastMessagePerCode[static_cast<int>(code)];
	}

	void cancel()
	{
		if (operation == nullptr)
			return;

		// Send a signal to the packer that it should stop further execution.
		// This method only sends a signal and returns immediately - in 
		// particular returning from this method doesn’t indicate that the 
		// packer already stopped the operation. After executing the cancel 
		// method you can expect that the call to the entry method will return
		// in a very short time (possibly with the return code set to CANCELLED).
		operation->cancel();
	}
};

void opExecutorMessageHandler(void* m_pMessageHandlerData, UvpMessageT* pMsg)
{
	// This handler is called every time the packer sends a message to the application.
	// Simply pass the message to the underlaying executor object.
	reinterpret_cast<UvpOpExecutorT*>(m_pMessageHandlerData)->handleMessage(pMsg);
}

// In place translation for matrix m
void translate_in_place(CLxMatrix4& m, float x, float y, float z)
{
	LXtVector4 t = { x, y, z, 0.0 };

	// For each row in matrix, increment the translate xyzw with row dot t
	for (int i = 0; i < 3; i++) {
		LXtVector4 r = { m[i][0], m[i][1], m[i][2], m[i][3] };
		m.set(i, 3, m[i][3] + LXx_V4DOT(t, r));
	}
}

// anisotropic scaling of matrix m
void scale_aniso(CLxMatrix4& m, float x, float y, float z)
{
	m.set(
		m[0][0] * x, m[0][1] * y, m[0][2] * z, m[0][3],
		m[1][0] * x, m[1][1] * y, m[1][2] * z, m[1][3],
		m[2][0] * x, m[2][1] * y, m[2][2] * z, m[2][3],
		m[3][0] * x, m[3][1] * y, m[3][2] * z, m[3][3]
	);
}

// get result of vector * matrix multiplication
void mat4x4_mul_vec4(LXtVector4& result, const LXtMatrix4 m, const LXtVector4 v)
{
	for (int i = 0; i < 4; i++)
	{
		result[i] = LXx_V4DOT(m[i], v);
	}
}

void islandSolutionToMatrix(const UvpIslandPackSolutionT& islandSolution, CLxMatrix4& mat)
{	// Set matrix used to transform UVs of the given island in order to apply
	// a packing result	
	mat.setToIdentity();

	// Move the uv island and apply scale in xy, 
	translate_in_place(mat, islandSolution.m_PostScaleOffset[0], islandSolution.m_PostScaleOffset[1], 0.0);
	scale_aniso(mat, 1.0 / islandSolution.m_Scale, 1.0 / islandSolution.m_Scale, 1.0);

	// Move the islands again, likely to compensate for scaling,
	translate_in_place(mat, islandSolution.m_Offset[0], islandSolution.m_Offset[1], 0.0);

	// Move the islands to prepare for rotation,
	translate_in_place(mat, islandSolution.m_Pivot[0], islandSolution.m_Pivot[1], 0.0);

	// Apply the rotation
	CLxVector z = { 0, 0, islandSolution.m_Angle };
	mat = mat * CLxMatrix4(z, LXi_ROTORD_XYZ);

	// Move the islands back after rotation,
	translate_in_place(mat, -islandSolution.m_Pivot[0], -islandSolution.m_Pivot[1], 0.0);

	scale_aniso(mat, islandSolution.m_PreScale, islandSolution.m_PreScale, 1.0);
}

// |======================================================================|
// | UV Packmaster related stuff should now be implemented, below is Modo |
// |======================================================================|

#define SRVNAME_COMMAND	"uvp.pack" // Define for our command name,

// Polygon Visitor to iterate over each face, can be replaced by simple iterating
// learned that the visitors are only better used when you only want to iterate selected etc...
class CVisitor : public CLxImpl_AbstractVisitor
{
public:
	// Accessors and the meshmap id we want to get uv values from
	CLxUser_Polygon* polygon;
	CLxUser_Point* point;
	LXtMeshMapID meshmapID;

	// Using a map only to check we don't add duplicate uv coords
	std::unordered_map<UvVertT, int, UvVertHashT, UvVertEqualT> vertPointerMap;

	// Containers to transfer the data to the uv packer later.
	std::vector<UvVertT> m_VertArray;
	std::vector<UvFaceT> m_FaceArray;

	CVisitor(CLxUser_Polygon* polygon, CLxUser_Point* point, LXtMeshMapID meshmapID) : polygon(polygon), point(point), meshmapID(meshmapID) {}

	// For each polygon, call this function,
	LxResult Evaluate() LXx_OVERRIDE
	{
		unsigned vertexIndex = 0;
		unsigned vertexCount = 0;
		check(polygon->VertexCount(&vertexCount));

		LXtFVector2 texcoord;
		LXtPointID pointID;

		LXtFVector position;

		// Store to the polygon index to face array
		int index = 0;
		check(polygon->Index(&index));

		this->m_FaceArray.emplace_back(index);
		UvFaceT& face = this->m_FaceArray.back();

		auto &faceVerts = face.m_Verts;
		faceVerts.reserve((int)vertexCount);
		
		for (int i = 0; i < vertexCount; i++)
		{
			// Get point in polygon,
			polygon->VertexByIndex(i, &pointID);
			point->Select(pointID);

			UvVertT uv;

			// Get the UV value for point in polygon
			LxResult result;
			result = polygon->MapEvaluate(meshmapID, pointID, texcoord);

			// If the UV was not mapped, abort enumeration
			if (result != LXe_OK)
				return LXe_FAILED;

			uv.m_UvCoords[0] = texcoord[0];
			uv.m_UvCoords[1] = texcoord[1];

			// Store the vertex index,
			point->Index(&vertexIndex);
			uv.m_ControlId = vertexIndex;

			// Get the vertex position,
			point->Pos(position);
			uv.m_Vert3dCoords[0] = position[0];
			uv.m_Vert3dCoords[1] = position[1];
			uv.m_Vert3dCoords[2] = position[2];

			// Push unique uvs to vectors,
			auto it = this->vertPointerMap.find(uv);
			size_t newVertIdx;
			if (it == this->vertPointerMap.end())
			{
				// It is the first time we see such a vertex - add it to the array
				newVertIdx = this->m_VertArray.size();
				this->vertPointerMap[uv] = newVertIdx;
				this->m_VertArray.emplace_back(uv);
			}
			else
			{
				// Such a vertex has already been added to the array.
				newVertIdx = (*it).second;
			}

			faceVerts.pushBack(newVertIdx);
		}

		return LXe_OK;
	}
};

class CCommand : public CLxBasicCommand
{
public:
	CCommand();

	LxResult cmd_DialogInit() LXx_OVERRIDE;

	int basic_CmdFlags() LXx_OVERRIDE;
	bool basic_Enable(CLxUser_Message& msg) LXx_OVERRIDE;
	void basic_Execute(unsigned flags);

	void cmd_error(LxResult rc, const char* message);
};

// Initialize the command, creating the arguments
CCommand::CCommand()
{
	dyna_Add("stretch", LXsTYPE_BOOLEAN);
	dyna_Add("orient", LXsTYPE_BOOLEAN);

	dyna_Add("margin", LXsTYPE_FLOAT);

	dyna_Add("pixelMargin", LXsTYPE_FLOAT);
	dyna_Add("pixelPadding", LXsTYPE_FLOAT);
	dyna_Add("pixelMarginTextureSize", LXsTYPE_INTEGER);
}

// Set default values for the command dialog
LxResult CCommand::cmd_DialogInit()
{
	attr_SetBool(0, true); // stretch
	attr_SetBool(1, true); // orient

	attr_SetFlt(2, 0.003); // margin

	attr_SetFlt(3, 0.0); // pixelMargin
	attr_SetFlt(4, 0.0); // pixelPadding
	attr_SetInt(5, 2048); // pixelMarginTextureSize

	return LXe_OK;
}

int CCommand::basic_CmdFlags()
{
	return LXfCMD_MODEL | LXfCMD_UNDO;
}


// Make sure the command is disabled with no active layers
bool CCommand::basic_Enable(CLxUser_Message& msg)
{
	int flags = 0;
	unsigned count;
	CLxUser_LayerService layer_service;

	check(layer_service.SetScene(0));
	check(layer_service.Count(&count));

	for (unsigned i = 0; i < count; i++)
	{
		check(layer_service.Flags(i, &flags));
		if (flags & LXf_LAYERSCAN_ACTIVE)
			return true;
	}

	return false;
}

void CCommand::basic_Execute(unsigned flags)
{
	UvpOperationInputT uvpInput;

	// Set some parameters by default
	// documentation: https://uvpackmaster.com/sdkdoc/70-packer-operations/20-pack/
	uvpInput.m_pDeviceId = "cpu";
	uvpInput.m_Opcode = UVP_OPCODE::PACK;

	// When stretch is set to true, the packer will scale islands during packing.
	// If UV islands can’t fit into the packing box, the NO_SPACE code 
	// will be returned by the operation.
	bool stretch = dyna_Bool(0, true);

	// if allow stretch == true, set fixed scale false
	uvpInput.m_FixedScale = !(bool)stretch;

	// If orient is false, do not allow packer to rotate the islands.
	bool orient = dyna_Bool(1, true);
	if (orient == 0)
	{
		uvpInput.m_RotationStep = 0;
		uvpInput.m_PrerotDisable = true;
	}

	// Determines the distance between islands after packing. The margin
	// distance is scaled by a certain factor after packing is done, that is
	// why the margin specified by this parameter is not exactly preserved.
	float margin = dyna_Float(2, 0.003);

	// Determines the distance between UV islands in pixels of the texture. 
	// A margin defined using this parameter is exact (in contrast to the 
	// m_Margin member). This parameter is only used if its value is greater
	// than 0 (in such a case the m_Margin option is ignored and this parameter
	// is used to determine distance between UV islands).
	float pixelMargin = dyna_Float(3, 0.0);

	// Determines the distance in pixels between UV islands and the packing
	// box border. This option is only used if m_PixelMargin is enabled. 
	// Setting m_PixelPadding to 0 means the feature will be ignored and pixel
	// padding will be equal to the half of m_PixelMargin.
	float pixelPadding = dyna_Float(4, 0.0);

	// Specifies the size of the texture the packed UV map will be used with. 
	// It allows proper application of the m_PixelMargin/ m_PixelPadding 
	// values during the packing process.
	int pixelMarginTextureSize = dyna_Int(5, 2048);

	// If user set the pixel margin, margin will be ignored.
	uvpInput.m_Margin = margin;

	uvpInput.m_PixelMargin = pixelMargin;
	uvpInput.m_PixelPadding = pixelPadding;
	uvpInput.m_PixelMarginTextureSize = pixelMarginTextureSize;

	// If set to true, the packer will automatically scale UV islands 
	// before packing so that the average texel density is the same 
	// for every island.
	uvpInput.m_NormalizeIslands = true;

	CLxUser_StdDialogService dialog_service;
	CLxUser_LayerService layer_service;
	CLxUser_LayerScan scan;

	// Accessors for LX SDK
	CLxUser_Mesh mesh;
	CLxUser_Point point;
	CLxUser_Polygon polygon;
	CLxUser_MeshMap meshmap;

	// Current layer index and total layer count
	unsigned index, count;

	// Get all selected layers
	check(layer_service.ScanAllocate(LXf_LAYERSCAN_EDIT, scan));
	check(scan.Count(&count));

	// For each layer,
	for (index = 0; index < count; index++)
	{
		// Get the editable mesh for current layer
		check(scan.MeshEdit(index, mesh));

		// Get the Accessors for Point, Polygon and Meshmap
		check(point.fromMesh(mesh));
		check(polygon.fromMesh(mesh));
		check(meshmap.fromMesh(mesh));

		// Get the default UV Map
		check(meshmap.SelectByName(LXi_VMAP_TEXTUREUV, "Texture"));

		// Create our simple visitor and enumerate for each polygon
		CVisitor visitor(&polygon, &point, meshmap.ID());
		LxResult visitor_result;
		visitor_result = polygon.Enum(&visitor, LXiMARK_ANY, 0);

		// Fail if we find any face-verts which haven't been mapped,
		if (visitor_result != LXe_OK)
			cmd_error(LXe_FAILED, "unmappedUV");

		// Copies over the harvested UV data to the UV Packer
		if (visitor.m_FaceArray.size() > 0)
		{
			uvpInput.m_UvData.m_FaceCount = visitor.m_FaceArray.size();
			uvpInput.m_UvData.m_pFaceArray = visitor.m_FaceArray.data();
		}

		if (visitor.m_VertArray.size() > 0)
		{
			uvpInput.m_UvData.m_VertCount = visitor.m_VertArray.size();
			uvpInput.m_UvData.m_pVertArray = visitor.m_VertArray.data();
		}

		// Initialize a progress bar for the user
		CLxUser_Monitor monitor;
		dialog_service.MonitorAllocate("Packing", monitor);
		monitor.Init(100);

		// Set debug to false for release,
		#ifdef _DEBUG
		bool debugMode = true;
		#else
		bool debugMode = false;
		#endif

		UvpOpExecutorT opExecutor(debugMode);

		// Run the execute method in another thread to not block main thread,
		// see execute method for more details...
		auto future = std::async(std::bind(&UvpOpExecutorT::execute, &opExecutor, uvpInput));

		// Keep track of progress on this thread,
		unsigned progress = 0;

		// Update every poll to see if user aborted the monitor progress, 
		bool bUserAborted = false;

		// Poll the packer instance every 50ms to check on progress,
		// while we keep getting progress updates.
		while (progress < 100)
		{
			unsigned step = opExecutor.packing_progress - progress;
			bUserAborted = monitor.Step(step);
			progress += step;

			if (bUserAborted)
			{
				opExecutor.cancel();
				break;
			}

			std::this_thread::sleep_for(std::chrono::milliseconds(50));
		}

		// Take the monitor the final step,
		monitor.Step(opExecutor.packing_progress - progress);

		// Get the result code from the packer instance, hopefully joins
		// the async thread also.
		UVP_ERRORCODE Result = future.get();

		// Release progress bar.
		dialog_service.MonitorRelease();

		// Switch on the result and return error messages defined as a 
		// message table in our config, see index.cfg
		switch (Result) {
		case UVP_ERRORCODE::SUCCESS:
			// All went fine, we likely don't have to report back anything
			break;
		case UVP_ERRORCODE::CANCELLED:
			cmd_error(LXe_ABORT, "uvpAborted");
			break;
		case UVP_ERRORCODE::NO_SPACE:
			// We have likely restricted the packer from scaling the 
			// islands, and it failed to fit them inside 0->1 uv range.
			cmd_error(LXe_FAILED, "uvpNoSpace");
			break;
		default:
			// Default to our "generic" error
			cmd_error(LXe_FAILED, "uvpFailed");
		}

		// fail if we did not recieve any solution,
		if (!opExecutor.getLastMessage(UvpMessageT::MESSAGE_CODE::ISLANDS) || !opExecutor.getLastMessage(UvpMessageT::MESSAGE_CODE::PACK_SOLUTION))
		{
			cmd_error(LXe_FAILED, "uvpMsgNotFound");
		}

		const UvpIslandsMessageT* pIslandsMsg = static_cast<const UvpIslandsMessageT*>(opExecutor.getLastMessage(UvpMessageT::MESSAGE_CODE::ISLANDS));
		const UvpPackSolutionMessageT* pPackSolutionMsg = static_cast<const UvpPackSolutionMessageT*>(opExecutor.getLastMessage(UvpMessageT::MESSAGE_CODE::PACK_SOLUTION));

		// Following the FBX example this is the apply function,
		std::vector<LXtFVector2> transformedUVs(visitor.m_VertArray.size());

		// Initially copy the original UV coordinates.
		for (int i = 0; i < visitor.m_VertArray.size(); i++)
		{
			const auto& origVert = visitor.m_VertArray[i];
			transformedUVs[i][0] = origVert.m_UvCoords[0];
			transformedUVs[i][1] = origVert.m_UvCoords[1];
		}

		// Transform UV coordinates accordingly
		const auto& islands = pIslandsMsg->m_Islands;
		for (const UvpIslandPackSolutionT& islandSolution : pPackSolutionMsg->m_IslandSolutions)
		{
			const IdxArrayT& island = islands[islandSolution.m_IslandIdx];
			CLxMatrix4 solutionMatrix;
			islandSolutionToMatrix(islandSolution, solutionMatrix);

			for (int faceId : island)
			{
				const UvFaceT& face = visitor.m_FaceArray[faceId];

				for (int vertIdx : face.m_Verts)
				{
					const UvVertT& origVert = visitor.m_VertArray[vertIdx];
					LXtVector4 inputUv = { origVert.m_UvCoords[0], origVert.m_UvCoords[1], 0.0, 1.0 };
					LXtVector4 transformedUv;

					mat4x4_mul_vec4(transformedUv, solutionMatrix, inputUv);

					transformedUVs[vertIdx][0] = transformedUv[0] / transformedUv[3];
					transformedUVs[vertIdx][1] = transformedUv[1] / transformedUv[3];
				}
			}
		}

		// This is where we then want to start setting the UV values ...
		for (const UvFaceT& uvFace : visitor.m_FaceArray)
		{
			polygon.SelectByIndex(uvFace.m_FaceId);
			for (const int vertIdx : uvFace.m_Verts)
			{
				const UvVertT& UvVert = visitor.m_VertArray[vertIdx];
				point.SelectByIndex(UvVert.m_ControlId);
				polygon.SetMapValue(point.ID(), meshmap.ID(), transformedUVs[vertIdx]);
			}
		}

		scan.SetMeshChange(index, LXf_MESHEDIT_MAP_UV);
	}

	scan.Apply();
}

// Basically attempting to do the same as CLxCommand::cmd_error
void CCommand::cmd_error(LxResult rc, const char* key)
{
	basic_Message().SetMsg("uvp.pack", key);
	throw(rc);
}

void initialize()
{
	CLxGenericPolymorph* srv;

	srv = new CLxPolymorph<CCommand>;
	srv->AddInterface(new CLxIfc_Command<CCommand>);
	srv->AddInterface(new CLxIfc_Attributes<CCommand>);
	srv->AddInterface(new CLxIfc_AttributesUI<CCommand>);
	lx::AddServer(SRVNAME_COMMAND, srv);
}