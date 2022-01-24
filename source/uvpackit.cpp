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
			// https://uvpackmaster.com/sdkdoc/40-uv-map-format/
			const char* pValidationResult = uvpInput.validate();

			// This runtime error will be caught inside the ccommand::execute when getting result from future,
			if (pValidationResult)
			{
				throw std::runtime_error("UVP Operation input validation failed: " + std::string(pValidationResult));
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

class CCommand : public CLxBasicCommand
{
public:
	CCommand();

	LxResult cmd_DialogInit() LXx_OVERRIDE;

	int basic_CmdFlags() LXx_OVERRIDE;
	bool basic_Enable(CLxUser_Message& msg) LXx_OVERRIDE;
	void basic_Execute(unsigned flags);

	void cmd_error(LxResult rc, const char* message);

	bool selectedPolygons();
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

	dyna_Add("normalizeIslands", LXsTYPE_BOOLEAN);

	dyna_Add("renderInvalid", LXsTYPE_BOOLEAN);
	dyna_SetFlags(7, LXfCMDARG_OPTIONAL);
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

// Utility method for checking if we have any currently selected polygons
bool CCommand::selectedPolygons()
{
	bool result = false;

	// Initiate the selection service and an array with polygon type and a null,
	CLxUser_SelectionService selection_service;
	LXtID4 selection_types[2];
	
	selection_types[0] = selection_service.LookupType(LXsSELTYP_POLYGON);
	selection_types[1] = 0;

	// Check the service for currently active selection type,
	LXtID4 current_type = selection_service.CurrentType(selection_types);

	// If we are in polygon mode and have polygons selected, result should be set to true
	if (current_type == LXiSEL_POLYGON)
		result = selection_service.Count(LXiSEL_POLYGON);

	return result;
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
	uvpInput.m_NormalizeIslands = dyna_Bool(6, false);

	// If users are in Polygon mode, and have polygons selected, assume they want to pack
	// the selected polygons into pre-existing packing solution.
	bool bArePolygonsSelected = selectedPolygons();
	uvpInput.m_PackToOthers = bArePolygonsSelected;

	// If m_ProcessedUnselected is set to false (the default state), then the 
	// SELECTED flag of the UV faces is ignored by the packer and every island
	// is considered as selected (the application doesn’t have to set this flag
	// in such a case).
	uvpInput.m_ProcessUnselected = bArePolygonsSelected; // Required so we check unselected

	// Optionally, render invalid UVs to better show users how to satisfy the packer.
	if(dyna_IsSet(7))
		uvpInput.m_RenderInvalidIslands = dyna_Bool(7, false);

	// Set debug to false for release,
	#ifdef _DEBUG
	bool debugMode = true;
	#else
	bool debugMode = false;
	#endif

	UvpOpExecutorT opExecutor(debugMode);

	CLxUser_StdDialogService dialog_service;
	CLxUser_LayerService layer_service;
	CLxUser_LayerScan scan;

	// Accessors for LX SDK
	CLxUser_Mesh mesh;
	CLxUser_Point point;
	CLxUser_Polygon polygon;
	CLxUser_MeshMap vmap;

	LXtFVector2 texcoords;
	LXtFVector position;

	// vectors for the data,
	std::unordered_map<UvVertT, int, UvVertHashT, UvVertEqualT> uv_map;  // Using a map only to check we don't add duplicate uv coords

	// Containers to transfer the data to the uv packer later.
	std::vector<UvVertT> m_VertArray;
	std::vector<UvFaceT> m_FaceArray;

	// UVP expects face id's as integer, while Modo have a typedef that can be cast to unsigned
	// and I don't want to bother with converting
	int uvp_face_index = 0;

	// Lookup tables to match Modo's IDs with whatever we tell UVP
	std::unordered_map<LXtPolygonID, int> polygon_map;
	std::unordered_map<int, LXtPointID> point_map;

	// Create the flag to test accessors for selection,
	CLxUser_MeshService mesh_service;
	unsigned mode;
	check(mesh_service.ModeCompose("select", NULL, &mode));

	// Iterate over all selected meshes,
	CLxUser_LayerScan selected_layers;
	unsigned selected_layers_count;
	check(layer_service.ScanAllocate(LXf_LAYERSCAN_ACTIVE | LXf_LAYERSCAN_MARKPOLYS, selected_layers));
	check(selected_layers.Count(&selected_layers_count));
	for (unsigned layer_index = 0; layer_index < selected_layers_count; layer_index++)
	{
		// Get mesh,
		check(selected_layers.BaseMeshByIndex(layer_index, mesh));

		// Get accessors for point, poly and vmap
		check(point.fromMesh(mesh));
		check(polygon.fromMesh(mesh));
		check(vmap.fromMesh(mesh));

		// Get the vmap, if not successful, skip layer
		LxResult uv_lookup = vmap.SelectByName(LXi_VMAP_TEXTUREUV, "Texture");
		if (uv_lookup != LXe_OK)
			continue;

		LXtMeshMapID vmap_id = vmap.ID();

		// For each polygon, get uv values for uvp
		unsigned polygon_count;
		mesh.PolygonCount(&polygon_count);
		for (unsigned polygon_index = 0; polygon_index < polygon_count; polygon_index++)
		{
			// Change the currently active polygon and get it's ID
			polygon.SelectByIndex(polygon_index);
			LXtPolygonID polygon_id = polygon.ID();

			// Store the Polygon ID so we can access and set the 
			std::pair<LXtPolygonID, int> polygon_id_lookup(polygon_id, uvp_face_index);
			polygon_map.insert(polygon_id_lookup);

			// Get the number of vertices for this polygon,
			unsigned vertex_count;
			polygon.VertexCount(&vertex_count);

			// Check if the polygon is selected
			CLxResult polygon_selected = polygon.TestMarks(mode);

			// Create the UVP Face, the counter uvp_polygon_id will act as the ID that we have mapped to Modo's IDs
			// https://uvpackmaster.com/sdkdoc/10-classes/50-uvfacet/
			m_FaceArray.emplace_back(uvp_face_index);
			UvFaceT& face = m_FaceArray.back();
			if (polygon_selected.isTrue())
				face.m_InputFlags = 1; // Sets the selection flag, couldn't find how to properly set it using the enum `uvpcore::UVP_FACE_INPUT_FLAGS::SELECTED`
			face.m_Verts.reserve((SizeT)vertex_count);

			// For each face vertex, get the texcoord values
			for (unsigned vertex_index = 0; vertex_index < vertex_count; vertex_index++)
			{
				LXtPointID point_id;
				polygon.VertexByIndex(vertex_index, &point_id);

				unsigned point_index;
				point.Index(&point_index);

				// Get the UV coordinates for polygon vertex
				polygon.MapEvaluate(vmap_id, point_id, texcoords);

				// And the positional data
				point.Select(point_id);
				point.Pos(position);

				// Create vertex and copy values for uvp, comments below are from docs
				// https://uvpackmaster.com/sdkdoc/10-classes/40-uvvertt/
				UvVertT uvp_vertex;

				// UV coordinates of the given UV vertex. This field must always be 
				// initialized by the application.
				uvp_vertex.m_UvCoords[0] = texcoords[0];
				uvp_vertex.m_UvCoords[1] = texcoords[1];

				// An integer value which is internally ignored by the packer, so the 
				// application may initialize it according to its needs. In particular 
				// this field might be used when building m_pVertArray in order to 
				// distinguish two UV vertices which have the same UV coordinates, but 
				// correspond to two different 3D vertices. Check the Sample application
				// code for a usage example - it initializes the m_ControlId field with 
				// an index of the corresponding 3d vertex in order to avoid duplicated 
				// vertices in a UV face.
				uvp_vertex.m_ControlId = reinterpret_cast<int>(point_id);

				// 3d coordinates of the 3d vertex corresponding to the given UV vertex. 
				// Currently this field is only used when m_NormalizeIslands parameter
				// is set to true. Otherwise it is ignored by the packer, hence it doesn’t
				// have to be initialized by the application.
				uvp_vertex.m_Vert3dCoords[0] = position[0];
				uvp_vertex.m_Vert3dCoords[1] = position[1];
				uvp_vertex.m_Vert3dCoords[2] = position[2];

				// Check for duplicate entries, as we iterate over each polygon they are likely to have vertices which
				// have same uv and positional values.
				auto iterator = uv_map.find(uvp_vertex);
				size_t uvp_vert_index;
				if (iterator == uv_map.end()) 
				{
					uvp_vert_index = m_VertArray.size(); // Get the size to update which index we're on.
					uv_map[uvp_vertex] = uvp_vert_index; // Add the pair uvp_vertex, uvp_vert_index to the uv_map
					m_VertArray.emplace_back(uvp_vertex);// Add the current UvVertT to the back of the array
					point_map[uvp_vert_index] = point_id;// Store so we can get the point id for a uv vertex.
				}
				else 
				{	// For the found pair, get the second element, which is the previously stored index.
					uvp_vert_index = (*iterator).second;
				}

				face.m_Verts.pushBack(uvp_vert_index);
			}
			uvp_face_index++;
		}
	}
	selected_layers.Apply(); // If we don't apply, next layerscan will fail it seem,
	selected_layers.clear();
	selected_layers = NULL;

	// Transfer the collected data to uvp input
	if (m_FaceArray.size() > 0)
	{
		uvpInput.m_UvData.m_FaceCount = m_FaceArray.size();
		uvpInput.m_UvData.m_pFaceArray = m_FaceArray.data();
	}
	if (m_VertArray.size() > 0)
	{
		uvpInput.m_UvData.m_VertCount = m_VertArray.size();
		uvpInput.m_UvData.m_pVertArray = m_VertArray.data();
	}

	// Initialize a progress bar for the user
	CLxUser_Monitor monitor;
	dialog_service.MonitorAllocate("Packing", monitor);
	monitor.Init(100);

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
	UVP_ERRORCODE Result = UVP_ERRORCODE::GENERAL_ERROR;
	try
	{
		if (future.valid())
		{
			Result = future.get();
		}
	} // Should only raise an exception if we're running in debug
	catch (const std::exception & ex) {
		// Set up to create log entries,
		CLxUser_Log log;
		CLxUser_LogService log_service;
		CLxUser_LogEntry entry;

		// Get the Master Log,
		log_service.GetSubSystem(LXsLOG_LOGSYS, log);

		// Print the runtime error to log so we can read any validation errors.
		log_service.NewEntry(LXe_INFO, ex.what(), entry);

		log.AddEntry(entry);
	}

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
	case UVP_ERRORCODE::INVALID_ISLANDS:
		// If there are two UV faces in a single island with different values 
		// of the parameter specified, then such an island will be reported as
		// invalid and the operation will fail with an INVLAID_ISLANDS return
		// code.
		cmd_error(LXe_FAILED, "uvpInvalidIslands");
		break;
	case UVP_ERRORCODE::NO_SPACE:
		// We have likely restricted the packer from scaling the 
		// islands, and it failed to fit them inside 0->1 uv range.
		cmd_error(LXe_FAILED, "uvpNoSpace");
		break;
	case UVP_ERRORCODE::NO_VALID_STATIC_ISLAND:
		cmd_error(LXe_FAILED, "uvpNoValidStaticIsland");
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

	// Copy over the values from the original input to the new texcoords
	std::vector<LXtFVector2> solved_texcoords(m_VertArray.size());
	for (int i = 0; i < m_VertArray.size(); i++)
	{
		const UvVertT& origVert = m_VertArray[i];
		solved_texcoords[i][0] = origVert.m_UvCoords[0];
		solved_texcoords[i][1] = origVert.m_UvCoords[1];
	}
	
	// Apply the transforms for the packing solution,
	const auto& islands = pIslandsMsg->m_Islands;
	const auto& solutions = pPackSolutionMsg->m_IslandSolutions;
	for (const UvpIslandPackSolutionT& islandSolution : pPackSolutionMsg->m_IslandSolutions)
	{
		const IdxArrayT& island = islands[islandSolution.m_IslandIdx];

		// Given a solution from uvp, get 
		CLxMatrix4 solutionMatrix;
		islandSolutionToMatrix(islandSolution, solutionMatrix);

		for (int faceId : island)
		{
			const UvFaceT& face = m_FaceArray[faceId];

			for (int vertIdx : face.m_Verts)
			{
				const UvVertT& origVert = m_VertArray[vertIdx];
				LXtVector4 input_uv = { origVert.m_UvCoords[0], origVert.m_UvCoords[1], 0.0, 1.0 };
				LXtVector4 solved_uv;

				mat4x4_mul_vec4(solved_uv, solutionMatrix, input_uv);

				solved_texcoords[vertIdx][0] = solved_uv[0] / solved_uv[3];
				solved_texcoords[vertIdx][1] = solved_uv[1] / solved_uv[3];
			}
		}
	}

	CLxUser_LayerScan editable_layers;
	check(layer_service.ScanAllocate(LXf_LAYERSCAN_EDIT, editable_layers));
	unsigned editable_layer_count;
	editable_layers.Count(&editable_layer_count);
	for (unsigned layer_index = 0; layer_index < editable_layer_count; layer_index++)
	{
		check(editable_layers.EditMeshByIndex(layer_index, mesh));
		check(polygon.fromMesh(mesh));
		check(vmap.fromMesh(mesh));

		// Get the vmap, if not successful, skip layer
		CLxResult uv_lookup = vmap.SelectByName(LXi_VMAP_TEXTUREUV, "Texture");
		if (uv_lookup.fail())
			continue;

		// For each polygon, set the uv for selected polygons,
		unsigned polygon_count;
		mesh.PolygonCount(&polygon_count);
		for (unsigned polygon_index = 0; polygon_index < polygon_count; polygon_index++)
		{
			polygon.SelectByIndex(polygon_index);
			LXtPolygonID polygon_id = polygon.ID();

			// Just skip this polygon if not selected,
			CLxResult polygon_selected = polygon.TestMarks(mode);
			if (polygon_selected.isFalse())
				continue;

			// Find the index for the current polygon in the polygon map,
			// the second element should hold index to solved uv face.
			int face_index = -1;
			auto iterator = polygon_map.find(polygon_id);
			if (iterator != polygon_map.end())
				face_index = iterator->second;

			// For each vertex in face, set the solved uv coordinates. Duplicate checks should already been made so 
			const UvFaceT& uv_face = m_FaceArray[face_index];
			for (const int vert_index : uv_face.m_Verts)
			{
				const UvVertT& uv_vert = m_VertArray[vert_index];
				// Find the stored point id for the uv vertex and set the map value
				auto point_id_lookup = point_map.find(vert_index);
				if (point_id_lookup != point_map.end())
					check(polygon.SetMapValue((*point_id_lookup).second, vmap.ID(), solved_texcoords[vert_index]));
			}
		}
		// If a mesh is accessed for write, any edits made have to be signalled back to the mesh.
		mesh.SetMeshEdits(LXf_MESHEDIT_MAP_UV);
		// The mesh change bit mask should be set for all edited meshes before changes are applied.
		editable_layers.SetMeshChange(layer_index, LXf_MESHEDIT_MAP_UV);
		// performs the mesh edits, but does not terminate the scan.
		editable_layers.Update();
	}
	editable_layers.Apply();
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