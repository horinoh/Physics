#pragma once

#include "resource.h"

#include <numbers>

#include "../DX.h"
#include "../GltfSDK.h"
#include "Physics.h"

class ConstraintDX : public Gltf::SDK, public DX
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

	void PlaceRigidBodies() {
		//!< 動的オブジェクト配置
		{
			constexpr auto Radius = 0.5f;
			constexpr auto Y = 10.0f;

			const auto Box = Scene->Shapes.emplace_back(std::make_unique<Physics::ShapeBox>(Radius * 2.0f)).get();
			const auto Sp = Scene->Shapes.emplace_back(std::make_unique<Physics::ShapeSphere>(Radius)).get();

			//!< 距離コンストレイント (中央上)
			{
				const auto JntRootPos = LinAlg::Vec3(0.0f, 6.0f, 0.0f);
				constexpr auto JntLen = 1.1f;
				constexpr auto JntCount = 5;

				auto RbA = Scene->RigidBodies.emplace_back(std::make_unique<Physics::RigidBody>(Sp, 0.0f)).get();
				RbA->Position = JntRootPos;

				for (auto i = 0; i < JntCount; ++i) {
					auto RbB = Scene->RigidBodies.emplace_back(std::make_unique<Physics::RigidBody>(Sp, 1.0f)).get();
					RbB->Position = RbA->Position + LinAlg::Vec3::AxisX() * JntLen;

					Scene->Constraints.emplace_back(std::make_unique<Physics::ConstraintDistance>(RbA, RbB, RbA->Position));
					RbA = RbB;
				}
			}
			//!< 移動、回転ムーバーコンストレイント (中央下)
			{
				const auto Pos = LinAlg::Vec3(0.0f, -6.0f, 0.0f);

				auto Rb = Scene->RigidBodies.emplace_back(std::make_unique<Physics::RigidBody>(Box, 0.0f)).get();
				Rb->Position = Pos;

				Scene->Constraints.emplace_back(std::make_unique<Physics::ConstraintMoverUpDown>(Rb));
				Scene->Constraints.emplace_back(std::make_unique<Physics::ConstraintMoverRotateY>(Rb));

				Rb = Scene->RigidBodies.emplace_back(std::make_unique<Physics::RigidBody>(Box, 1.0f)).get();
				Rb->Position = Pos + LinAlg::Vec3::AxisY() * 5.2f;
			}
			//!< ヒンジコンストレイント (右)
			{
				const auto JntRootPos = LinAlg::Vec3(7.5f, 6.0f, 0.0f);
				constexpr auto JntLen = 1.2f;
				constexpr auto JntCount = 5;

				auto RbA = Scene->RigidBodies.emplace_back(std::make_unique<Physics::RigidBody>(Box, 0.0f)).get();
				RbA->Position = JntRootPos;

				for (auto i = 0; i < JntCount; ++i) {
					auto RbB = Scene->RigidBodies.emplace_back(std::make_unique<Physics::RigidBody>(Box, 1.0f)).get();
					RbB->Position = RbA->Position + LinAlg::Vec3::AxisZ() * JntLen;

					Scene->Constraints.emplace_back(std::make_unique<Physics::ConstraintHinge>(RbA, RbB, RbA->Position, LinAlg::Vec3::AxisX()));
					RbA = RbB;
				}
			}
#ifndef _DEBUG
			//!< コンストレイント (右下)
			{
				const auto Pos = LinAlg::Vec3(7.5f, -5.0f, 0.0f);

				auto Rb = Scene->RigidBodies.emplace_back(std::make_unique<Physics::RigidBody>(Box, 0.0f)).get();
				Rb->Position = Pos;
				Scene->Constraints.emplace_back(std::make_unique<Physics::ConstraintMoverRotateZ>(Rb));

				Rb = Scene->RigidBodies.emplace_back(std::make_unique<Physics::RigidBody>(Box, 0.0f)).get();
				Rb->Position = Pos - LinAlg::Vec3::AxisZ() + LinAlg::Vec3::AxisY() * 0.0f;
				Scene->Constraints.emplace_back(std::make_unique<Physics::ConstraintMoverRotateZ>(Rb));
				Rb = Scene->RigidBodies.emplace_back(std::make_unique<Physics::RigidBody>(Box, 0.0f)).get();
				Rb->Position = Pos + LinAlg::Vec3::AxisZ() + LinAlg::Vec3::AxisY() * 0.0f;
				Scene->Constraints.emplace_back(std::make_unique<Physics::ConstraintMoverRotateZ>(Rb));

				constexpr auto Div = 16;
				constexpr auto Radius = 2.5f;
				constexpr auto DRadian = std::numbers::pi_v<float> *2.0f / Div;

				auto Rb0 = Scene->RigidBodies.emplace_back(std::make_unique<Physics::RigidBody>(Box, 1.0f)).get();
				Rb0->Position = Pos + LinAlg::Vec3(Radius * std::cosf(0.0f), Radius * std::sin(0.0f), 0.0f);
				auto RbA = Rb0;
				for (auto i = 1; i < Div; ++i) {
					auto RbB = Scene->RigidBodies.emplace_back(std::make_unique<Physics::RigidBody>(Box, 1.0f)).get();
					const auto Rad = DRadian * i;
					RbB->Position = Pos + LinAlg::Vec3(Radius * std::cosf(Rad), Radius * std::sin(Rad), 0.0f);

					const auto Anchor = (RbA->Position + RbB->Position) * 0.5f;
					Scene->Constraints.emplace_back(std::make_unique<Physics::ConstraintDistance>(RbA, RbB, Anchor));
					RbA = RbB;
				}
				Scene->Constraints.emplace_back(std::make_unique<Physics::ConstraintDistance>(RbA, Rb0, RbA->Position));
			}
#endif
			//!< 角度制限ヒンジコンストレイント (左)
			{
				const auto JntRootPos = LinAlg::Vec3(-7.5f, 6.0f, 0.0f);
				constexpr auto JntLen = 1.2f;
				constexpr auto JntCount = 5;

				auto RbA = Scene->RigidBodies.emplace_back(std::make_unique<Physics::RigidBody>(Box, 0.0f)).get();
				RbA->Position = JntRootPos;

				for (auto i = 0; i < JntCount; ++i) {
					auto RbB = Scene->RigidBodies.emplace_back(std::make_unique<Physics::RigidBody>(Box, 1.0f)).get();
					RbB->Position = RbA->Position + LinAlg::Vec3::AxisX() * JntLen;

					Scene->Constraints.emplace_back(std::make_unique<Physics::ConstraintHingeLimited>(RbA, RbB, RbA->Position, LinAlg::Vec3::AxisZ()));
					RbA = RbB;
				}
			}
			//!< ボールソケットコンストレイント (最右)
			{
				const auto Pos = LinAlg::Vec3(15.0f, 6.0f, 0.0f);

				auto RbA = Scene->RigidBodies.emplace_back(std::make_unique<Physics::RigidBody>(Sp, 0.0f)).get();
				RbA->Position = Pos;

				auto RbB = Scene->RigidBodies.emplace_back(std::make_unique<Physics::RigidBody>(Box, 1.0f)).get();
				RbB->Position = Pos - LinAlg::Vec3::AxisX() * 1.2f;

				Scene->Constraints.emplace_back(std::make_unique<Physics::ConstraintBallSocket>(RbA, RbB, RbA->Position, LinAlg::Vec3::AxisY()));
				//Scene->Constraints.emplace_back(std::make_unique<Physics::ConstraintBallSocketLimited>(RbA, RbB, RbA->Position, Math::Vec3::AxisY(), 135.0f, 135.0f));
			}
			//!< モーターコンストレイント (最左)
			{
				const auto Pos = LinAlg::Vec3(-15.0f, 6.0f, 0.0f);

				auto RbA = Scene->RigidBodies.emplace_back(std::make_unique<Physics::RigidBody>(Sp, 0.0f)).get();
				RbA->Position = Pos;

				auto RbB = Scene->RigidBodies.emplace_back(std::make_unique<Physics::RigidBody>(Box, 1.0f)).get();
				RbB->Position = Pos - LinAlg::Vec3::AxisY() * 1.2f;

				Scene->Constraints.emplace_back(std::make_unique <Physics::ConstraintMotor>(RbA, RbB, RbA->Position, LinAlg::Vec3::AxisY(), 2.0f));
			}
		}
	}

	virtual void OnCreate(HWND hWnd, HINSTANCE hInstance, LPCWSTR Title) override {
		DX::OnCreate(hWnd, hInstance, Title);

		Scene = new Physics::Scene();
		PlaceRigidBodies();
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
		const auto CA = COM_PTR_GET(DirectCommandAllocators[0]);
		const auto CL = COM_PTR_GET(DirectCommandLists[0]);
		const auto CQ = COM_PTR_GET(GraphicsCommandQueue);

		Meshes.emplace_back();
		Load(ASSET_PATH / "Box.glb");
		Meshes.emplace_back();
		Load(ASSET_PATH / "Sphere.glb");

		const auto& Box = Meshes[0];
		VertexBuffers.emplace_back().Create(COM_PTR_GET(Device), TotalSizeOf(Box.Vertices), sizeof(Box.Vertices[0]));
		UploadResource UploadVertex_Box;
		UploadVertex_Box.Create(COM_PTR_GET(Device), TotalSizeOf(Box.Vertices), std::data(Box.Vertices));

		VertexBuffers.emplace_back().Create(COM_PTR_GET(Device), TotalSizeOf(Box.Normals), sizeof(Box.Normals[0]));
		UploadResource UploadNormal_Box;
		UploadNormal_Box.Create(COM_PTR_GET(Device), TotalSizeOf(Box.Normals), std::data(Box.Normals));

		IndexBuffers.emplace_back().Create(COM_PTR_GET(Device), TotalSizeOf(Box.Indices), DXGI_FORMAT_R32_UINT);
		UploadResource UploadIndex_Box;
		UploadIndex_Box.Create(COM_PTR_GET(Device), TotalSizeOf(Box.Indices), std::data(Box.Indices));

		const D3D12_DRAW_INDEXED_ARGUMENTS DIA_Box = {
			.IndexCountPerInstance = static_cast<UINT32>(std::size(Box.Indices)),
			.InstanceCount = _countof(WorldBuffer.Instances0),
			.StartIndexLocation = 0,
			.BaseVertexLocation = 0,
			.StartInstanceLocation = 0
		};
		IndirectBuffers.emplace_back().Create(COM_PTR_GET(Device), DIA_Box);
		UploadResource UploadIndirect_Box;
		UploadIndirect_Box.Create(COM_PTR_GET(Device), sizeof(DIA_Box), &DIA_Box);

		const auto& Sphere = Meshes[1];
		VertexBuffers.emplace_back().Create(COM_PTR_GET(Device), TotalSizeOf(Sphere.Vertices), sizeof(Sphere.Vertices[0]));
		UploadResource UploadVertex_Sphere;
		UploadVertex_Sphere.Create(COM_PTR_GET(Device), TotalSizeOf(Sphere.Vertices), std::data(Sphere.Vertices));

		VertexBuffers.emplace_back().Create(COM_PTR_GET(Device), TotalSizeOf(Sphere.Normals), sizeof(Sphere.Normals[0]));
		UploadResource UploadNormal_Sphere;
		UploadNormal_Sphere.Create(COM_PTR_GET(Device), TotalSizeOf(Sphere.Normals), std::data(Sphere.Normals));

		IndexBuffers.emplace_back().Create(COM_PTR_GET(Device), TotalSizeOf(Sphere.Indices), DXGI_FORMAT_R32_UINT);
		UploadResource UploadIndex_Sphere;
		UploadIndex_Sphere.Create(COM_PTR_GET(Device), TotalSizeOf(Sphere.Indices), std::data(Sphere.Indices));

		const D3D12_DRAW_INDEXED_ARGUMENTS DIA_Sphere = {
			.IndexCountPerInstance = static_cast<UINT32>(std::size(Sphere.Indices)),
			.InstanceCount = _countof(WorldBuffer.Instances1),
			.StartIndexLocation = 0,
			.BaseVertexLocation = 0,
			.StartInstanceLocation = 0
		};
		IndirectBuffers.emplace_back().Create(COM_PTR_GET(Device), DIA_Sphere);
		UploadResource UploadIndirect_Sphere;
		UploadIndirect_Sphere.Create(COM_PTR_GET(Device), sizeof(DIA_Sphere), &DIA_Sphere);


		VERIFY_SUCCEEDED(CL->Reset(CA, nullptr)); {
			VertexBuffers[0].PopulateCopyCommand(CL, TotalSizeOf(Box.Vertices), COM_PTR_GET(UploadVertex_Box.Resource));
			VertexBuffers[1].PopulateCopyCommand(CL, TotalSizeOf(Box.Normals), COM_PTR_GET(UploadNormal_Box.Resource));
			IndexBuffers[0].PopulateCopyCommand(CL, TotalSizeOf(Box.Indices), COM_PTR_GET(UploadIndex_Box.Resource));
			IndirectBuffers[0].PopulateCopyCommand(CL, sizeof(DIA_Box), COM_PTR_GET(UploadIndirect_Box.Resource));

			VertexBuffers[2].PopulateCopyCommand(CL, TotalSizeOf(Sphere.Vertices), COM_PTR_GET(UploadVertex_Sphere.Resource));
			VertexBuffers[3].PopulateCopyCommand(CL, TotalSizeOf(Sphere.Normals), COM_PTR_GET(UploadNormal_Sphere.Resource));
			IndexBuffers[1].PopulateCopyCommand(CL, TotalSizeOf(Sphere.Indices), COM_PTR_GET(UploadIndex_Sphere.Resource));
			IndirectBuffers[1].PopulateCopyCommand(CL, sizeof(DIA_Sphere), COM_PTR_GET(UploadIndirect_Sphere.Resource));
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

		std::vector<COM_PTR<ID3DBlob>> SBs;
		VERIFY_SUCCEEDED(D3DReadFileToBlob(std::data((std::filesystem::path(".") / "ConstraintDX.vs.cso").wstring()), COM_PTR_PUT(SBs.emplace_back())));
		VERIFY_SUCCEEDED(D3DReadFileToBlob(std::data((std::filesystem::path(".") / "ConstraintDX.ps.cso").wstring()), COM_PTR_PUT(SBs.emplace_back())));
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

		std::vector<COM_PTR<ID3DBlob>> SBs_1;
		VERIFY_SUCCEEDED(D3DReadFileToBlob(std::data((std::filesystem::path(".") / "ConstraintDX_1.vs.cso").wstring()), COM_PTR_PUT(SBs_1.emplace_back())));
		VERIFY_SUCCEEDED(D3DReadFileToBlob(std::data((std::filesystem::path(".") / "ConstraintDX.ps.cso").wstring()), COM_PTR_PUT(SBs_1.emplace_back())));
		const std::array SBCs_1 = {
			D3D12_SHADER_BYTECODE({.pShaderBytecode = SBs_1[0]->GetBufferPointer(), .BytecodeLength = SBs_1[0]->GetBufferSize() }),
			D3D12_SHADER_BYTECODE({.pShaderBytecode = SBs_1[1]->GetBufferPointer(), .BytecodeLength = SBs_1[1]->GetBufferSize() }),
		};
		DX::CreatePipelineState_VsPs_Input(PipelineStates[1], COM_PTR_GET(RootSignatures[0]), D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE, RD, TRUE, IEDs, SBCs_1);

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
		const auto PS0 = COM_PTR_GET(PipelineStates[0]), PS1 = COM_PTR_GET(PipelineStates[1]);
		const auto BCL = COM_PTR_GET(BundleCommandLists[i]);
		const auto BCA = COM_PTR_GET(BundleCommandAllocators[0]);

		VERIFY_SUCCEEDED(BCL->Reset(BCA, PS0));
		{
			BCL->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

			BCL->SetPipelineState(PS0);
			{
				const std::array VBVs = { VertexBuffers[0].View, VertexBuffers[1].View };
				BCL->IASetVertexBuffers(0, static_cast<UINT>(std::size(VBVs)), std::data(VBVs));
				BCL->IASetIndexBuffer(&IndexBuffers[0].View);
				BCL->ExecuteIndirect(COM_PTR_GET(IndirectBuffers[0].CommandSignature), 1, COM_PTR_GET(IndirectBuffers[0].Resource), 0, nullptr, 0);
			}
			BCL->SetPipelineState(PS1);
			{
				const std::array VBVs = { VertexBuffers[2].View, VertexBuffers[3].View };
				BCL->IASetVertexBuffers(0, static_cast<UINT>(std::size(VBVs)), std::data(VBVs));
				BCL->IASetIndexBuffer(&IndexBuffers[1].View);
				BCL->ExecuteIndirect(COM_PTR_GET(IndirectBuffers[1].CommandSignature), 1, COM_PTR_GET(IndirectBuffers[1].Resource), 0, nullptr, 0);
			}
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
				const auto Rb = Scene->RigidBodies[i].get();
				const auto Pos = DirectX::XMLoadFloat4(reinterpret_cast<const DirectX::XMFLOAT4*>(static_cast<const float*>(Rb->Position)));
				const auto Rot = DirectX::XMLoadFloat4(reinterpret_cast<const DirectX::XMFLOAT4*>(static_cast<const float*>(Rb->Rotation)));
				if (Rb->Shape->GetShapeType() == Physics::Shape::SHAPE_TYPE::BOX) {
					const auto Scl = static_cast<const Physics::ShapeBox*>(Rb->Shape)->CalcExtent() * 0.5f;
					if (i0 < _countof(WorldBuffer.Instances0)) {
						DirectX::XMStoreFloat4x4(&WorldBuffer.Instances0[i0++].World, DirectX::XMMatrixScaling(Scl.X(), Scl.Y(), Scl.Z()) * DirectX::XMMatrixRotationQuaternion(Rot) * DirectX::XMMatrixTranslationFromVector(Pos));
					}
				}
				if (Rb->Shape->GetShapeType() == Physics::Shape::SHAPE_TYPE::SPHERE) {
					const auto Scl = static_cast<const Physics::ShapeSphere*>(Rb->Shape)->Radius;
					if (i1 < _countof(WorldBuffer.Instances1)) {
						DirectX::XMStoreFloat4x4(&WorldBuffer.Instances1[i1++].World, DirectX::XMMatrixScaling(Scl, Scl, Scl) * DirectX::XMMatrixRotationQuaternion(Rot) * DirectX::XMMatrixTranslationFromVector(Pos));
					}
				}
			}
		}
	}
	virtual void UpdateViewProjectionBuffer() {
		const auto Pos = DirectX::XMVectorSet(0.0f, 15.0f, 30.0f, 1.0f);
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
	struct MESH {
		std::vector<UINT32> Indices;
		std::vector<DirectX::XMFLOAT3> Vertices;
		std::vector<DirectX::XMFLOAT3> Normals;
	};
	std::vector<MESH> Meshes;

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
