#pragma once

#include "resource.h"

#include <numbers>

#include "../DX.h"
#include "../GltfSDK.h"
#include "Physics.h"

#define USE_MESH
#ifdef USE_MESH
#define USE_MESH_HULL
#endif

class ConvexHullDX : public Gltf::SDK, public DX
{
public:
	virtual void Process() override {
		for (const auto& i : Document.meshes.Elements()) {
			for (const auto& j : i.primitives) {
				switch (j.mode)
				{
				case Microsoft::glTF::MeshMode::MESH_TRIANGLES:
					break;				
				default:
					__debugbreak();
					break;
				}

				auto& Mesh = Meshes.back();
				if (empty(Mesh.Indices)) {
					if (Document.accessors.Has(j.indicesAccessorId)) {
						const auto& Accessor = Document.accessors.Get(j.indicesAccessorId);
						switch (Accessor.componentType)
						{
						case Microsoft::glTF::ComponentType::COMPONENT_UNSIGNED_SHORT:
							switch (Accessor.type)
							{
							case Microsoft::glTF::AccessorType::TYPE_SCALAR:
							{
								std::vector<UINT32> Indices16(Accessor.count);
								std::ranges::copy(ResourceReader->ReadBinaryData<uint16_t>(Document, Accessor), std::begin(Indices16));

								Mesh.Indices.reserve(Accessor.count);
								for (auto i : Indices16) {
									Mesh.Indices.emplace_back(i);
								}
							}
							break;
							default: break;
							}
							break;
						case Microsoft::glTF::ComponentType::COMPONENT_UNSIGNED_INT:
							switch (Accessor.type)
							{
							case Microsoft::glTF::AccessorType::TYPE_SCALAR:
							{
								Mesh.Indices.resize(Accessor.count);
								std::ranges::copy(ResourceReader->ReadBinaryData<uint32_t>(Document, Accessor), std::begin(Mesh.Indices));
							}
							break;
							default: break;
							}
							break;
						default: break;
						}
					}
				}

				std::string AccessorId;
				if (empty(Mesh.Vertices)) {
					if (j.TryGetAttributeAccessorId(Microsoft::glTF::ACCESSOR_POSITION, AccessorId))
					{
						const auto& Accessor = Document.accessors.Get(AccessorId);
						Mesh.Vertices.resize(Accessor.count);
						switch (Accessor.componentType)
						{
						case Microsoft::glTF::ComponentType::COMPONENT_FLOAT:
							switch (Accessor.type)
							{
							case Microsoft::glTF::AccessorType::TYPE_VEC3:
							{
								std::memcpy(std::data(Mesh.Vertices), std::data(ResourceReader->ReadBinaryData<float>(Document, Accessor)), TotalSizeOf(Mesh.Vertices));
							}
							break;
							default: break;
							}
							break;
						default: break;
						}
					}
				}
				if (empty(Mesh.Normals)) {
					if (j.TryGetAttributeAccessorId(Microsoft::glTF::ACCESSOR_NORMAL, AccessorId))
					{
						const auto& Accessor = Document.accessors.Get(AccessorId);
						Mesh.Normals.resize(Accessor.count);
						switch (Accessor.componentType)
						{
						case Microsoft::glTF::ComponentType::COMPONENT_FLOAT:
							switch (Accessor.type)
							{
							case Microsoft::glTF::AccessorType::TYPE_VEC3:
							{
								std::memcpy(std::data(Mesh.Normals), std::data(ResourceReader->ReadBinaryData<float>(Document, Accessor)), TotalSizeOf(Mesh.Normals));
							}
							break;
							default: break;
							}
							break;
						default: break;
						}
					}
				}
			}
		}
	}

	void PlaceRigidBodies(const std::vector<LinAlg::Vec3>& Vertices) {
		//!< 動的オブジェクト配置
		{
			constexpr auto Offset = 3.0f * 1.5f;
			constexpr auto Y = 10.0f;

			Scene->Shapes.emplace_back(std::make_unique<Physics::ShapeConvex>(Vertices));

#ifdef _DEBUG
			const auto n = 3;
#else
			const auto n = 5;
#endif
			const auto n2 = n >> 1;
			for (auto x = 0; x < n; ++x) {
				for (auto z = 0; z < n; ++z) {
					auto Rb = Scene->RigidBodies.emplace_back(std::make_unique<Physics::RigidBody>(Scene->Shapes.back().get(), 1.0f)).get();
					Rb->Position = LinAlg::Vec3(static_cast<float>(x - n2) * Offset, Y, static_cast<float>(z - n2) * Offset);
					Rb->Rotation = LinAlg::Quat(LinAlg::Vec3::AxisX(), LinAlg::ToRadian(90.0f));
				}
			}
		}
		//!< 静的オブジェクト配置
		{
			constexpr auto Y = -20.0f;

			//!< 拡大する
			std::vector<LinAlg::Vec3> ExpandedVertices(std::size(Vertices));
			std::ranges::transform(Vertices, std::back_inserter(ExpandedVertices), [&](const auto& i) { return i * FloorScale; });

			Scene->Shapes.emplace_back(std::make_unique<Physics::ShapeConvex>(ExpandedVertices));

			auto Rb = Scene->RigidBodies.emplace_back(std::make_unique<Physics::RigidBody>(Scene->Shapes.back().get(), 0.0f)).get();
			Rb->Position = LinAlg::Vec3::AxisY() * Y;
			Rb->Rotation = LinAlg::Quat(LinAlg::Vec3::AxisX(), LinAlg::ToRadian(270.0f));
			Rb->Elasticity = 0.99f;
		}
	}

	virtual void OnCreate(HWND hWnd, HINSTANCE hInstance, LPCWSTR Title) override {
#ifdef _DEBUG
		Collision::SignedVolumeTest();
#endif
		Scene = new Physics::Scene();

		DX::OnCreate(hWnd, hInstance, Title);
	}
	virtual void OnDestroy(HWND hWnd, HINSTANCE hInstance) override {
		DX::OnDestroy(hWnd, hInstance);

		if (nullptr != Scene) {
			delete Scene;
		}
	}
	virtual void OnTimer(HWND hWnd, HINSTANCE hInstance) override {
		DX::OnTimer(hWnd, hInstance);

		if (IsUpdate()) {
			if (nullptr != Scene) {
				Scene->Update(1.0f / 60.0f);
			}
		}
	}
	
	virtual void DrawFrame(const UINT i) override {
		UpdateWorldBuffer();
		CopyToUploadResource(COM_PTR_GET(ConstantBuffers[i].Resource), RoundUp256(sizeof(WorldBuffer)), &WorldBuffer);
	}

	virtual void CreateCommandList() override {
		DX::CreateCommandList();
		DX::CreateBundleCommandList(std::size(SwapChainBackBuffers));
	}
	virtual void CreateGeometry() override {
		std::vector<LinAlg::Vec3> Vec3s;
#ifdef USE_MESH
		Meshes.emplace_back();
		Load(GLTF_PATH / "SuzanneMorphSparse" / "glTF-Binary" / "SuzanneMorphSparse.glb");
		Vec3s.reserve(std::size(Meshes.back().Vertices));
		for (auto& i : Meshes.back().Vertices) { Vec3s.emplace_back(LinAlg::Vec3({ i.x, i.y, i.z })); }
#else
		//!< ダイアモンド形状
		std::vector<Math::Vec3> ShapeVert;
		Physics::CreateVertices_Diamond(ShapeVert);
		std::ranges::copy(ShapeVert, std::back_inserter(Vec3s));
#endif

		PlaceRigidBodies(Vec3s);
		const auto Convex = static_cast<const Physics::ShapeConvex*>(Scene->Shapes.front().get());
		if (nullptr != Convex) {
			for (auto& i : Convex->Vertices) {
				Vertices_CH.emplace_back(DirectX::XMFLOAT3(i.X(), i.Y(), i.Z()));
			}
			for (auto i : Convex->Indices) {
				Indices_CH.emplace_back(i[0]);
				Indices_CH.emplace_back(i[1]);
				Indices_CH.emplace_back(i[2]);
			}
		}

		const auto CA = COM_PTR_GET(DirectCommandAllocators[0]);
		const auto CL = COM_PTR_GET(DirectCommandLists[0]);
		const auto CQ = COM_PTR_GET(GraphicsCommandQueue);

		Meshes.emplace_back();
		Load(ASSET_PATH / "Box.glb");

#ifdef USE_MESH
		const auto Mesh = Meshes[0];
		VertexBuffers.emplace_back().Create(COM_PTR_GET(Device), TotalSizeOf(Mesh.Vertices), sizeof(Mesh.Vertices[0]));
		UploadResource UploadVertex;
		UploadVertex.Create(COM_PTR_GET(Device), TotalSizeOf(Mesh.Vertices), std::data(Mesh.Vertices));
		VertexBuffers.emplace_back().Create(COM_PTR_GET(Device), TotalSizeOf(Mesh.Normals), sizeof(Mesh.Normals[0]));
		UploadResource UploadNormal;
		UploadNormal.Create(COM_PTR_GET(Device), TotalSizeOf(Mesh.Normals), std::data(Mesh.Normals));
		IndexBuffers.emplace_back().Create(COM_PTR_GET(Device), TotalSizeOf(Mesh.Indices), DXGI_FORMAT_R32_UINT);
		UploadResource UploadIndex;
		UploadIndex.Create(COM_PTR_GET(Device), TotalSizeOf(Mesh.Indices), std::data(Mesh.Indices));
		const D3D12_DRAW_INDEXED_ARGUMENTS DIA = { 
			.IndexCountPerInstance = static_cast<UINT32>(std::size(Mesh.Indices)),
			.InstanceCount = _countof(WorldBuffer.Instances0),
			.StartIndexLocation = 0,
			.BaseVertexLocation = 0,
			.StartInstanceLocation = 0
		};
		IndirectBuffers.emplace_back().Create(COM_PTR_GET(Device), DIA);
		UploadResource UploadIndirect;
		UploadIndirect.Create(COM_PTR_GET(Device), sizeof(DIA), &DIA);
#endif

		VertexBuffers.emplace_back().Create(COM_PTR_GET(Device), TotalSizeOf(Vertices_CH), sizeof(Vertices_CH[0]));
		UploadResource UploadVertex_CH;
		UploadVertex_CH.Create(COM_PTR_GET(Device), TotalSizeOf(Vertices_CH), std::data(Vertices_CH));
		IndexBuffers.emplace_back().Create(COM_PTR_GET(Device), TotalSizeOf(Indices_CH), DXGI_FORMAT_R32_UINT);
		UploadResource UploadIndex_CH;
		UploadIndex_CH.Create(COM_PTR_GET(Device), TotalSizeOf(Indices_CH), std::data(Indices_CH));
		const D3D12_DRAW_INDEXED_ARGUMENTS DIA_CH = {
			.IndexCountPerInstance = static_cast<UINT32>(std::size(Indices_CH)),
			.InstanceCount = _countof(WorldBuffer.Instances0),
			.StartIndexLocation = 0,
			.BaseVertexLocation = 0,
			.StartInstanceLocation = 0
		};
		IndirectBuffers.emplace_back().Create(COM_PTR_GET(Device), DIA_CH);
		UploadResource UploadIndirect_CH;
		UploadIndirect_CH.Create(COM_PTR_GET(Device), sizeof(DIA_CH), &DIA_CH);

#ifdef USE_MESH
		const auto& Floor = Meshes[1];
#else
		const auto& Floor = Meshes[0];
#endif
		VertexBuffers.emplace_back().Create(COM_PTR_GET(Device), TotalSizeOf(Floor.Vertices), sizeof(Floor.Vertices[0]));
		UploadResource UploadVertex_FLR;
		UploadVertex_FLR.Create(COM_PTR_GET(Device), TotalSizeOf(Floor.Vertices), std::data(Floor.Vertices));
		VertexBuffers.emplace_back().Create(COM_PTR_GET(Device), TotalSizeOf(Floor.Normals), sizeof(Floor.Normals[0]));
		UploadResource UploadNormal_FLR;
		UploadNormal_FLR.Create(COM_PTR_GET(Device), TotalSizeOf(Floor.Normals), std::data(Floor.Normals));
		IndexBuffers.emplace_back().Create(COM_PTR_GET(Device), TotalSizeOf(Floor.Indices), DXGI_FORMAT_R32_UINT);
		UploadResource UploadIndex_FLR;
		UploadIndex_FLR.Create(COM_PTR_GET(Device), TotalSizeOf(Floor.Indices), std::data(Floor.Indices));
		const D3D12_DRAW_INDEXED_ARGUMENTS DIA_FLR = {
			.IndexCountPerInstance = static_cast<UINT32>(std::size(Floor.Indices)),
			.InstanceCount = _countof(WorldBuffer.Instances1),
			.StartIndexLocation = 0,
			.BaseVertexLocation = 0,
			.StartInstanceLocation = 0
		};
		IndirectBuffers.emplace_back().Create(COM_PTR_GET(Device), DIA_FLR);
		UploadResource UploadIndirect_FLR;
		UploadIndirect_FLR.Create(COM_PTR_GET(Device), sizeof(DIA_FLR), &DIA_FLR);

		VERIFY_SUCCEEDED(CL->Reset(CA, nullptr)); {
#ifdef USE_MESH
			const auto Mesh = Meshes[0];
			VertexBuffers[0].PopulateCopyCommand(CL, TotalSizeOf(Mesh.Vertices), COM_PTR_GET(UploadVertex.Resource));
			VertexBuffers[1].PopulateCopyCommand(CL, TotalSizeOf(Mesh.Normals), COM_PTR_GET(UploadNormal.Resource));
			IndexBuffers[0].PopulateCopyCommand(CL, TotalSizeOf(Mesh.Indices), COM_PTR_GET(UploadIndex.Resource));
			IndirectBuffers[0].PopulateCopyCommand(CL, sizeof(DIA), COM_PTR_GET(UploadIndirect.Resource));

			VertexBuffers[2].PopulateCopyCommand(CL, TotalSizeOf(Vertices_CH), COM_PTR_GET(UploadVertex_CH.Resource));
			IndexBuffers[1].PopulateCopyCommand(CL, TotalSizeOf(Indices_CH), COM_PTR_GET(UploadIndex_CH.Resource));
			IndirectBuffers[1].PopulateCopyCommand(CL, sizeof(DIA_CH), COM_PTR_GET(UploadIndirect_CH.Resource));

			const auto Floor = Meshes[1];
			VertexBuffers[3].PopulateCopyCommand(CL, TotalSizeOf(Floor.Vertices), COM_PTR_GET(UploadVertex_FLR.Resource));
			VertexBuffers[4].PopulateCopyCommand(CL, TotalSizeOf(Floor.Normals), COM_PTR_GET(UploadNormal_FLR.Resource));
			IndexBuffers[2].PopulateCopyCommand(CL, TotalSizeOf(Floor.Indices), COM_PTR_GET(UploadIndex_FLR.Resource));
			IndirectBuffers[2].PopulateCopyCommand(CL, sizeof(DIA), COM_PTR_GET(UploadIndirect_FLR.Resource));
#else
			VertexBuffers[0].PopulateCopyCommand(CL, TotalSizeOf(Vertices_CH), COM_PTR_GET(UploadVertex_CH.Resource));
			IndexBuffers[0].PopulateCopyCommand(CL, TotalSizeOf(Indices_CH), COM_PTR_GET(UploadIndex_CH.Resource));
			IndirectBuffers[0].PopulateCopyCommand(CL, sizeof(DIA_CH), COM_PTR_GET(UploadIndirect_CH.Resource));

			const auto Floor = Meshes[0];
			VertexBuffers[1].PopulateCopyCommand(CL, TotalSizeOf(Floor.Vertices), COM_PTR_GET(UploadVertex_FLR.Resource));
			VertexBuffers[2].PopulateCopyCommand(CL, TotalSizeOf(Floor.Normals), COM_PTR_GET(UploadNormal_FLR.Resource));
			IndexBuffers[1].PopulateCopyCommand(CL, TotalSizeOf(Floor.Indices), COM_PTR_GET(UploadIndex_FLR.Resource));
			IndirectBuffers[1].PopulateCopyCommand(CL, sizeof(DIA_FLR), COM_PTR_GET(UploadIndirect_FLR.Resource));
#endif
		} VERIFY_SUCCEEDED(CL->Close());
		DX::ExecuteAndWait(CQ, CL, COM_PTR_GET(GraphicsFence));
	}
	virtual void CreateConstantBuffer() override {
		UpdateWorldBuffer();
		UpdateViewProjectionBuffer();

		for (const auto& i : SwapChainBackBuffers) {
			ConstantBuffers.emplace_back().Create(COM_PTR_GET(Device), sizeof(WorldBuffer));
			CopyToUploadResource(COM_PTR_GET(ConstantBuffers.back().Resource), RoundUp256(sizeof(WorldBuffer)), &WorldBuffer);
		}
		for (const auto& i : SwapChainBackBuffers) {
			ConstantBuffers.emplace_back().Create(COM_PTR_GET(Device), sizeof(ViewProjectionBuffer));
			CopyToUploadResource(COM_PTR_GET(ConstantBuffers.back().Resource), RoundUp256(sizeof(ViewProjectionBuffer)), &ViewProjectionBuffer);
		}
	}
	virtual void CreateTexture() override {
		DX::CreateTexture_Depth();
	}
	virtual void CreateRootSignature() override {
		COM_PTR<ID3DBlob> Blob;
		constexpr std::array DRs = {
			D3D12_DESCRIPTOR_RANGE1({
				.RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_CBV,
				.NumDescriptors = 2,
				.BaseShaderRegister = 0,
				.RegisterSpace = 0,
				.Flags = D3D12_DESCRIPTOR_RANGE_FLAG_NONE,
				.OffsetInDescriptorsFromTableStart = D3D12_DESCRIPTOR_RANGE_OFFSET_APPEND
			})
		};
		DX::SerializeRootSignature(Blob,
			{
				D3D12_ROOT_PARAMETER1({
					.ParameterType = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE,
					.DescriptorTable = D3D12_ROOT_DESCRIPTOR_TABLE1({.NumDescriptorRanges = static_cast<UINT>(std::size(DRs)), .pDescriptorRanges = std::data(DRs) }),
					.ShaderVisibility = D3D12_SHADER_VISIBILITY_VERTEX
				}),
			},
			{
			},
			D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT | SHADER_ROOT_ACCESS_VS);
		VERIFY_SUCCEEDED(Device->CreateRootSignature(0, Blob->GetBufferPointer(), Blob->GetBufferSize(), COM_PTR_UUIDOF_PUTVOID(RootSignatures.emplace_back())));
	}
	virtual void CreatePipelineState() override {
		PipelineStates.emplace_back();
		PipelineStates.emplace_back();
		PipelineStates.emplace_back();

		std::vector<COM_PTR<ID3DBlob>> SBs;
		VERIFY_SUCCEEDED(D3DReadFileToBlob(std::data((std::filesystem::path(".") / "ConvexHullDX.vs.cso").wstring()), COM_PTR_PUT(SBs.emplace_back())));
		VERIFY_SUCCEEDED(D3DReadFileToBlob(std::data((std::filesystem::path(".") / "ConvexHullDX.ps.cso").wstring()), COM_PTR_PUT(SBs.emplace_back())));
		const std::array SBCs = {
			D3D12_SHADER_BYTECODE({.pShaderBytecode = SBs[0]->GetBufferPointer(), .BytecodeLength = SBs[0]->GetBufferSize() }),
			D3D12_SHADER_BYTECODE({.pShaderBytecode = SBs[1]->GetBufferPointer(), .BytecodeLength = SBs[1]->GetBufferSize() }),
		};
		const std::vector IEDs = {
			D3D12_INPUT_ELEMENT_DESC({.SemanticName = "POSITION", .SemanticIndex = 0, .Format = DXGI_FORMAT_R32G32B32_FLOAT, .InputSlot = 0, .AlignedByteOffset = D3D12_APPEND_ALIGNED_ELEMENT, .InputSlotClass = D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, .InstanceDataStepRate = 0 }),
			D3D12_INPUT_ELEMENT_DESC({.SemanticName = "NORMAL", .SemanticIndex = 0, .Format = DXGI_FORMAT_R32G32B32_FLOAT, .InputSlot = 1, .AlignedByteOffset = D3D12_APPEND_ALIGNED_ELEMENT, .InputSlotClass = D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, .InstanceDataStepRate = 0 }),
		};
		constexpr D3D12_RASTERIZER_DESC RD = {
			.FillMode = D3D12_FILL_MODE_SOLID,
			.CullMode = D3D12_CULL_MODE_BACK, .FrontCounterClockwise = TRUE,
			.DepthBias = D3D12_DEFAULT_DEPTH_BIAS, .DepthBiasClamp = D3D12_DEFAULT_DEPTH_BIAS_CLAMP, .SlopeScaledDepthBias = D3D12_DEFAULT_SLOPE_SCALED_DEPTH_BIAS,
			.DepthClipEnable = TRUE,
			.MultisampleEnable = FALSE, .AntialiasedLineEnable = FALSE, .ForcedSampleCount = 0,
			.ConservativeRaster = D3D12_CONSERVATIVE_RASTERIZATION_MODE_OFF
		};
		DX::CreatePipelineState_VsPs_Input(PipelineStates[0], COM_PTR_GET(RootSignatures[0]), D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE, RD, TRUE, IEDs, SBCs);

		std::vector<COM_PTR<ID3DBlob>> SBs_CH;
		VERIFY_SUCCEEDED(D3DReadFileToBlob(std::data((std::filesystem::path(".") / "ConvexHullDX_CH.vs.cso").wstring()), COM_PTR_PUT(SBs_CH.emplace_back())));
		VERIFY_SUCCEEDED(D3DReadFileToBlob(std::data((std::filesystem::path(".") / "ConvexHullDX_CH.ps.cso").wstring()), COM_PTR_PUT(SBs_CH.emplace_back())));
		const std::array SBCs_CH = {
			D3D12_SHADER_BYTECODE({.pShaderBytecode = SBs_CH[0]->GetBufferPointer(), .BytecodeLength = SBs_CH[0]->GetBufferSize() }),
			D3D12_SHADER_BYTECODE({.pShaderBytecode = SBs_CH[1]->GetBufferPointer(), .BytecodeLength = SBs_CH[1]->GetBufferSize() }),
		};
		const std::vector IEDs_CH = {
			D3D12_INPUT_ELEMENT_DESC({.SemanticName = "POSITION", .SemanticIndex = 0, .Format = DXGI_FORMAT_R32G32B32_FLOAT, .InputSlot = 0, .AlignedByteOffset = D3D12_APPEND_ALIGNED_ELEMENT, .InputSlotClass = D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, .InstanceDataStepRate = 0 }),
		};
		constexpr D3D12_RASTERIZER_DESC RD_CH = {
			.FillMode = D3D12_FILL_MODE_WIREFRAME,
			.CullMode = D3D12_CULL_MODE_BACK, .FrontCounterClockwise = TRUE,
			.DepthBias = D3D12_DEFAULT_DEPTH_BIAS, .DepthBiasClamp = D3D12_DEFAULT_DEPTH_BIAS_CLAMP, .SlopeScaledDepthBias = D3D12_DEFAULT_SLOPE_SCALED_DEPTH_BIAS,
			.DepthClipEnable = TRUE,
			.MultisampleEnable = FALSE, .AntialiasedLineEnable = FALSE, .ForcedSampleCount = 0,
			.ConservativeRaster = D3D12_CONSERVATIVE_RASTERIZATION_MODE_OFF
		};
		DX::CreatePipelineState_VsPs_Input(PipelineStates[1], COM_PTR_GET(RootSignatures[0]), D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE, RD_CH, TRUE, IEDs_CH, SBCs_CH);

		std::vector<COM_PTR<ID3DBlob>> SBs_FLR;
		VERIFY_SUCCEEDED(D3DReadFileToBlob(std::data((std::filesystem::path(".") / "ConvexHullDX_FLR.vs.cso").wstring()), COM_PTR_PUT(SBs_FLR.emplace_back())));
		VERIFY_SUCCEEDED(D3DReadFileToBlob(std::data((std::filesystem::path(".") / "ConvexHullDX.ps.cso").wstring()), COM_PTR_PUT(SBs_FLR.emplace_back())));
		const std::array SBCs_FLR = {
			D3D12_SHADER_BYTECODE({.pShaderBytecode = SBs_FLR[0]->GetBufferPointer(), .BytecodeLength = SBs_FLR[0]->GetBufferSize() }),
			D3D12_SHADER_BYTECODE({.pShaderBytecode = SBs_FLR[1]->GetBufferPointer(), .BytecodeLength = SBs_FLR[1]->GetBufferSize() }),
		};
		DX::CreatePipelineState_VsPs_Input(PipelineStates[2], COM_PTR_GET(RootSignatures[0]), D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE, RD, TRUE, IEDs, SBCs_FLR);

		for (auto& i : Threads) { i.join(); }
		Threads.clear();
	}
	virtual void CreateDescriptor() override {
		{
			auto& Desc = DsvDescs.emplace_back();
			auto& Heap = Desc.first;
			auto& Handle = Desc.second;

			const D3D12_DESCRIPTOR_HEAP_DESC DHD = { .Type = D3D12_DESCRIPTOR_HEAP_TYPE_DSV, .NumDescriptors = 1, .Flags = D3D12_DESCRIPTOR_HEAP_FLAG_NONE, .NodeMask = 0 };
			VERIFY_SUCCEEDED(Device->CreateDescriptorHeap(&DHD, COM_PTR_UUIDOF_PUTVOID(Heap)));

			auto CDH = Heap->GetCPUDescriptorHandleForHeapStart();
			const auto IncSize = Device->GetDescriptorHandleIncrementSize(Heap->GetDesc().Type);

			const auto& Tex = DepthTextures[0];
			Device->CreateDepthStencilView(COM_PTR_GET(Tex.Resource), &Tex.DSV, CDH);
			Handle.emplace_back(CDH);
			CDH.ptr += IncSize;
		}

		{
			const auto BackBufferCount = std::size(SwapChainBackBuffers);
			const auto DescCount = 2;

			const auto CB0Index = 0;
			const auto CB1Index = BackBufferCount;

			for (auto i = 0; i < BackBufferCount; ++i) {
				auto& Desc = CbvSrvUavDescs.emplace_back();
				auto& Heap = Desc.first;
				auto& Handle = Desc.second;

				const D3D12_DESCRIPTOR_HEAP_DESC DHD = {
					.Type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV,
					.NumDescriptors = DescCount,
					.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE,
					.NodeMask = 0
				};
				VERIFY_SUCCEEDED(Device->CreateDescriptorHeap(&DHD, COM_PTR_UUIDOF_PUTVOID(Heap)));
				auto CDH = Heap->GetCPUDescriptorHandleForHeapStart();
				auto GDH = Heap->GetGPUDescriptorHandleForHeapStart();
				const auto IncSize = Device->GetDescriptorHandleIncrementSize(Heap->GetDesc().Type);

				{
					const auto& CB = ConstantBuffers[CB0Index + i];
					const D3D12_CONSTANT_BUFFER_VIEW_DESC CBVD = {
						.BufferLocation = CB.Resource->GetGPUVirtualAddress(),
						.SizeInBytes = static_cast<UINT>(CB.Resource->GetDesc().Width)
					};
					Device->CreateConstantBufferView(&CBVD, CDH);
					Handle.emplace_back(GDH);
					CDH.ptr += IncSize;
					GDH.ptr += IncSize;
				}
				{
					const auto& CB = ConstantBuffers[CB1Index + i];
					const D3D12_CONSTANT_BUFFER_VIEW_DESC CBVD = {
						.BufferLocation = CB.Resource->GetGPUVirtualAddress(),
						.SizeInBytes = static_cast<UINT>(CB.Resource->GetDesc().Width)
					};
					Device->CreateConstantBufferView(&CBVD, CDH);
					Handle.emplace_back(GDH);
					CDH.ptr += IncSize;
					GDH.ptr += IncSize;
				}
			}
		}
	}
	virtual void PopulateBundleCommandList(const size_t i) override {
		const auto PS0 = COM_PTR_GET(PipelineStates[0]);
		const auto PS1 = COM_PTR_GET(PipelineStates[1]);
		const auto PS2 = COM_PTR_GET(PipelineStates[2]);
		const auto BCL = COM_PTR_GET(BundleCommandLists[i]);
		const auto BCA = COM_PTR_GET(BundleCommandAllocators[0]);

		VERIFY_SUCCEEDED(BCL->Reset(BCA, PS0));
		{
			BCL->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

#ifdef USE_MESH
			//!< メッシュ
			{
				BCL->SetPipelineState(PS0);
				const std::array VBVs = { VertexBuffers[0].View, VertexBuffers[1].View };
				BCL->IASetVertexBuffers(0, static_cast<UINT>(std::size(VBVs)), std::data(VBVs));
				BCL->IASetIndexBuffer(&IndexBuffers[0].View);
				BCL->ExecuteIndirect(COM_PTR_GET(IndirectBuffers[0].CommandSignature), 1, COM_PTR_GET(IndirectBuffers[0].Resource), 0, nullptr, 0);
			}
#ifdef USE_MESH_HULL
			//!< 凸包
			{
				BCL->SetPipelineState(PS1);
				const std::array VBVs_CH = { VertexBuffers[2].View };
				BCL->IASetVertexBuffers(0, static_cast<UINT>(std::size(VBVs_CH)), std::data(VBVs_CH));
				BCL->IASetIndexBuffer(&IndexBuffers[1].View);
				BCL->ExecuteIndirect(COM_PTR_GET(IndirectBuffers[1].CommandSignature), 1, COM_PTR_GET(IndirectBuffers[1].Resource), 0, nullptr, 0);
			}
#endif
			//!< フロア
			{
				BCL->SetPipelineState(PS2);
				const std::array VBVs_FLR = { VertexBuffers[3].View, VertexBuffers[4].View };
				BCL->IASetVertexBuffers(0, static_cast<UINT>(std::size(VBVs_FLR)), std::data(VBVs_FLR));
				BCL->IASetIndexBuffer(&IndexBuffers[2].View);
				BCL->ExecuteIndirect(COM_PTR_GET(IndirectBuffers[2].CommandSignature), 1, COM_PTR_GET(IndirectBuffers[2].Resource), 0, nullptr, 0);
			}
#else
			//!< 凸包
			{
				BCL->SetPipelineState(PS1);
				const std::array VBVs_CH = { VertexBuffers[0].View };
				BCL->IASetVertexBuffers(0, static_cast<UINT>(std::size(VBVs_CH)), std::data(VBVs_CH));
				BCL->IASetIndexBuffer(&IndexBuffers[0].View);
				BCL->ExecuteIndirect(COM_PTR_GET(IndirectBuffers[0].CommandSignature), 1, COM_PTR_GET(IndirectBuffers[0].Resource), 0, nullptr, 0);
			}
			//!< フロア
			{
				BCL->SetPipelineState(PS2);
				const std::array VBVs_FLR = { VertexBuffers[1].View, VertexBuffers[2].View };
				BCL->IASetVertexBuffers(0, static_cast<UINT>(std::size(VBVs_FLR)), std::data(VBVs_FLR));
				BCL->IASetIndexBuffer(&IndexBuffers[1].View);
				BCL->ExecuteIndirect(COM_PTR_GET(IndirectBuffers[1].CommandSignature), 1, COM_PTR_GET(IndirectBuffers[1].Resource), 0, nullptr, 0);
			}
#endif
		}
		VERIFY_SUCCEEDED(BCL->Close());
	}
	virtual void PopulateCommandList(const size_t i) override {
		const auto BCL = COM_PTR_GET(BundleCommandLists[i]);
		const auto DCL = COM_PTR_GET(DirectCommandLists[i]);
		const auto DCA = COM_PTR_GET(DirectCommandAllocators[0]);

		VERIFY_SUCCEEDED(DCL->Reset(DCA, nullptr));
		{
			DCL->SetGraphicsRootSignature(COM_PTR_GET(RootSignatures[0]));

			DCL->RSSetViewports(static_cast<UINT>(std::size(Viewports)), std::data(Viewports));
			DCL->RSSetScissorRects(static_cast<UINT>(std::size(ScissorRects)), std::data(ScissorRects));

			const auto SCR = COM_PTR_GET(SwapChainBackBuffers[i].Resource);
			ResourceBarrier(DCL, SCR, D3D12_RESOURCE_STATE_PRESENT, D3D12_RESOURCE_STATE_RENDER_TARGET);
			{
				const auto& HandleDSV = DsvDescs[0].second;

				constexpr std::array<D3D12_RECT, 0> Rects = {};
				DCL->ClearRenderTargetView(SwapChainBackBuffers[i].Handle, DirectX::Colors::SkyBlue, static_cast<UINT>(std::size(Rects)), std::data(Rects));
				DCL->ClearDepthStencilView(HandleDSV[0], D3D12_CLEAR_FLAG_DEPTH, 1.0f, 0, static_cast<UINT>(std::size(Rects)), std::data(Rects));

				const std::array CHs = { SwapChainBackBuffers[i].Handle };
				DCL->OMSetRenderTargets(static_cast<UINT>(std::size(CHs)), std::data(CHs), FALSE, &HandleDSV[0]);

				{
					const auto& Desc = CbvSrvUavDescs[i];
					const auto& Heap = Desc.first;
					const auto& Handle = Desc.second;

					const std::array DHs = { COM_PTR_GET(Heap) };
					DCL->SetDescriptorHeaps(static_cast<UINT>(std::size(DHs)), std::data(DHs));

					DCL->SetGraphicsRootDescriptorTable(0, Handle[0]);
				}

				DCL->ExecuteBundle(BCL);
			}
			ResourceBarrier(DCL, SCR, D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_PRESENT);
		}
		VERIFY_SUCCEEDED(DCL->Close());
	}

	virtual void UpdateWorldBuffer() {
		if (nullptr != Scene) {
			for (auto i = 0, i0 = 0, i1 = 0; i < std::size(Scene->RigidBodies); ++i) {
				if (i < _countof(WorldBuffer.Instances0)) {
					const auto Rb = Scene->RigidBodies[i].get();
					const auto Pos = DirectX::XMLoadFloat4(reinterpret_cast<const DirectX::XMFLOAT4*>(static_cast<const float*>(Rb->Position)));
					const auto Rot = DirectX::XMLoadFloat4(reinterpret_cast<const DirectX::XMFLOAT4*>(static_cast<const float*>(Rb->Rotation)));
					const auto Scl = 0.0f == Rb->InvMass ? DirectX::XMMatrixScaling(FloorScale, FloorScale, FloorScale) : DirectX::XMMatrixScaling(1.0f, 1.0f, 1.0f);
					if (i0 < _countof(WorldBuffer.Instances0)) {
						DirectX::XMStoreFloat4x4(&WorldBuffer.Instances0[i0++].World, Scl * DirectX::XMMatrixRotationQuaternion(Rot) * DirectX::XMMatrixTranslationFromVector(Pos));
					}
				}
			}
		}
	}
	virtual void UpdateViewProjectionBuffer() {
		const auto Pos = DirectX::XMVectorSet(0.0f, 15.0f, 40.0f, 1.0f);
		const auto Tag = DirectX::XMVectorSet(0.0f, 0.0f, 0.0f, 1.0f);
		const auto Up = DirectX::XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f);
		const auto View = DirectX::XMMatrixLookAtRH(Pos, Tag, Up);

		constexpr auto Fov = 0.16f * std::numbers::pi_v<float>;
		const auto Aspect = static_cast<float>(Rect.right - Rect.left) / (Rect.bottom - Rect.top);
		constexpr auto ZFar = 100.0f;
		constexpr auto ZNear = ZFar * 0.0001f;
		const auto Projection = DirectX::XMMatrixPerspectiveFovRH(Fov, Aspect, ZNear, ZFar);

		DirectX::XMStoreFloat4x4(&ViewProjectionBuffer.ViewProjection, View * Projection);
	}

protected:
	const float FloorScale = 30.0f;

	struct MESH {
		std::vector<UINT32> Indices;
		std::vector<DirectX::XMFLOAT3> Vertices;
		std::vector<DirectX::XMFLOAT3> Normals;
	};
	std::vector<MESH> Meshes;

	std::vector<UINT32> Indices_CH;
	std::vector<DirectX::XMFLOAT3> Vertices_CH;

	Physics::Scene* Scene = nullptr;

	struct INSTANCE {
		DirectX::XMFLOAT4X4 World;
	};
	struct WORLD_BUFFER {
		INSTANCE Instances0[64];
		INSTANCE Instances1[64];
	};
	WORLD_BUFFER WorldBuffer; 
	struct VIEW_PROJECTION_BUFFER {
		DirectX::XMFLOAT4X4 ViewProjection;
	};
	VIEW_PROJECTION_BUFFER ViewProjectionBuffer;
};
