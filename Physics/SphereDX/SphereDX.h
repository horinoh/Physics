#pragma once

#include "resource.h"

#include <numbers>

#include "../DX.h"
#include "../GltfSDK.h"
#include "Physics.h"

class SphereDX : public Gltf::SDK, public DX
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

				if (empty(Indices)) {
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

								Indices.reserve(Accessor.count);
								for (auto i : Indices16) {
									Indices.emplace_back(i);
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
								Indices.resize(Accessor.count);
								std::ranges::copy(ResourceReader->ReadBinaryData<uint32_t>(Document, Accessor), std::begin(Indices));
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
				if (empty(Vertices)) {
					if (j.TryGetAttributeAccessorId(Microsoft::glTF::ACCESSOR_POSITION, AccessorId))
					{
						const auto& Accessor = Document.accessors.Get(AccessorId);
						Vertices.resize(Accessor.count);
						switch (Accessor.componentType)
						{
						case Microsoft::glTF::ComponentType::COMPONENT_FLOAT:
							switch (Accessor.type)
							{
							case Microsoft::glTF::AccessorType::TYPE_VEC3:
							{
								std::memcpy(data(Vertices), data(ResourceReader->ReadBinaryData<float>(Document, Accessor)), TotalSizeOf(Vertices));
							}
							break;
							default: break;
							}
							break;
						default: break;
						}
					}
				}
				if (empty(Normals)) {
					if (j.TryGetAttributeAccessorId(Microsoft::glTF::ACCESSOR_NORMAL, AccessorId))
					{
						const auto& Accessor = Document.accessors.Get(AccessorId);
						Normals.resize(Accessor.count);
						switch (Accessor.componentType)
						{
						case Microsoft::glTF::ComponentType::COMPONENT_FLOAT:
							switch (Accessor.type)
							{
							case Microsoft::glTF::AccessorType::TYPE_VEC3:
							{
								std::memcpy(data(Normals), data(ResourceReader->ReadBinaryData<float>(Document, Accessor)), TotalSizeOf(Normals));
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

			static_cast<Physics::ShapeSphere*>(Scene->Shapes.emplace_back(new Physics::ShapeSphere(Radius)))->Init();

			const auto n = 6;
			const auto n2 = n >> 1;
			for (auto x = 0; x < n; ++x) {
				for (auto z = 0; z < n; ++z) {
					auto Rb = Scene->RigidBodies.emplace_back(new Physics::RigidBody());
					Rb->Position = Math::Vec3(static_cast<float>(x - n2) * Radius * 2.0f * 1.5f, Y, static_cast<float>(z - n2) * Radius * 2.0f * 1.5f);
					Rb->Init(Scene->Shapes.back());
				}
			}
		}

		//!< 静的オブジェクト配置
		{
			constexpr auto Radius = 80.0f;
			constexpr auto Y = -Radius;

			static_cast<Physics::ShapeSphere*>(Scene->Shapes.emplace_back(new Physics::ShapeSphere(Radius)))->Init();

			const auto n = 3;
			const auto n2 = n >> 1;
			for (auto x = 0; x < n; ++x) {
				for (auto z = 0; z < n; ++z) {
					auto Rb = Scene->RigidBodies.emplace_back(new Physics::RigidBody());
					Rb->Position = Math::Vec3(static_cast<float>(x - n2) * Radius * 0.25f, Y, static_cast<float>(z - n2) * Radius * 0.25f);
					Rb->InvMass = 0;
					Rb->Elasticity = 0.99f;
					Rb->Init(Scene->Shapes.back());
				}
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
				Scene->Update(1.0f/60.0f);
			}
		}
	}
	virtual void DrawFrame(const UINT i) override {
		UpdateWorldBuffer();
		CopyToUploadResource(COM_PTR_GET(ConstantBuffers[i].Resource), RoundUp256(sizeof(WorldBuffer)), &WorldBuffer);
	}

	virtual void CreateCommandList() override {
		DX::CreateCommandList();
		DX::CreateBundleCommandList(size(SwapChainBackBuffers));
	}
	virtual void CreateGeometry() override {
		Load(ASSET_PATH / "Sphere.glb");

		const auto CA = COM_PTR_GET(DirectCommandAllocators[0]);
		const auto CL = COM_PTR_GET(DirectCommandLists[0]);
		const auto CQ = COM_PTR_GET(GraphicsCommandQueue);

		VertexBuffers.emplace_back().Create(COM_PTR_GET(Device), TotalSizeOf(Vertices), sizeof(Vertices[0]));
		UploadResource UploadVertex;
		UploadVertex.Create(COM_PTR_GET(Device), TotalSizeOf(Vertices), data(Vertices));

		VertexBuffers.emplace_back().Create(COM_PTR_GET(Device), TotalSizeOf(Normals), sizeof(Normals[0]));
		UploadResource UploadNormal;
		UploadNormal.Create(COM_PTR_GET(Device), TotalSizeOf(Normals), data(Normals));

		IndexBuffers.emplace_back().Create(COM_PTR_GET(Device), TotalSizeOf(Indices), DXGI_FORMAT_R32_UINT);
		UploadResource UploadIndex;
		UploadIndex.Create(COM_PTR_GET(Device), TotalSizeOf(Indices), data(Indices));

		const D3D12_DRAW_INDEXED_ARGUMENTS DIA = {
			.IndexCountPerInstance = static_cast<UINT32>(size(Indices)),
			.InstanceCount = _countof(WorldBuffer.Instances),
			.StartIndexLocation = 0,
			.BaseVertexLocation = 0,
			.StartInstanceLocation = 0
		};
		IndirectBuffers.emplace_back().Create(COM_PTR_GET(Device), DIA);
		UploadResource UploadIndirect;
		UploadIndirect.Create(COM_PTR_GET(Device), sizeof(DIA), &DIA);

		VERIFY_SUCCEEDED(CL->Reset(CA, nullptr)); {
			VertexBuffers[0].PopulateCopyCommand(CL, TotalSizeOf(Vertices), COM_PTR_GET(UploadVertex.Resource));
			VertexBuffers[1].PopulateCopyCommand(CL, TotalSizeOf(Normals), COM_PTR_GET(UploadNormal.Resource));
			IndexBuffers[0].PopulateCopyCommand(CL, TotalSizeOf(Indices), COM_PTR_GET(UploadIndex.Resource));
			IndirectBuffers[0].PopulateCopyCommand(CL, sizeof(DIA), COM_PTR_GET(UploadIndirect.Resource));
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
					.DescriptorTable = D3D12_ROOT_DESCRIPTOR_TABLE1({.NumDescriptorRanges = static_cast<UINT>(size(DRs)), .pDescriptorRanges = data(DRs) }),
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

		std::vector<COM_PTR<ID3DBlob>> SBs;
		VERIFY_SUCCEEDED(D3DReadFileToBlob(data((std::filesystem::path(".") / "SphereDX.vs.cso").wstring()), COM_PTR_PUT(SBs.emplace_back())));
		VERIFY_SUCCEEDED(D3DReadFileToBlob(data((std::filesystem::path(".") / "SphereDX.ps.cso").wstring()), COM_PTR_PUT(SBs.emplace_back())));
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
			const auto BackBufferCount = size(SwapChainBackBuffers);
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
		const auto PS = COM_PTR_GET(PipelineStates[0]);
		const auto BCL = COM_PTR_GET(BundleCommandLists[i]);
		const auto BCA = COM_PTR_GET(BundleCommandAllocators[0]);

		VERIFY_SUCCEEDED(BCL->Reset(BCA, PS));
		{
			BCL->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

			const std::array VBVs = { VertexBuffers[0].View, VertexBuffers[1].View };
			BCL->IASetVertexBuffers(0, static_cast<UINT>(size(VBVs)), data(VBVs));
			BCL->IASetIndexBuffer(&IndexBuffers[0].View);
			BCL->ExecuteIndirect(COM_PTR_GET(IndirectBuffers[0].CommandSignature), 1, COM_PTR_GET(IndirectBuffers[0].Resource), 0, nullptr, 0);
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

			DCL->RSSetViewports(static_cast<UINT>(size(Viewports)), data(Viewports));
			DCL->RSSetScissorRects(static_cast<UINT>(size(ScissorRects)), data(ScissorRects));

			const auto SCR = COM_PTR_GET(SwapChainBackBuffers[i].Resource);
			ResourceBarrier(DCL, SCR, D3D12_RESOURCE_STATE_PRESENT, D3D12_RESOURCE_STATE_RENDER_TARGET);
			{
				const auto& HandleDSV = DsvDescs[0].second;

				constexpr std::array<D3D12_RECT, 0> Rects = {};
				DCL->ClearRenderTargetView(SwapChainBackBuffers[i].Handle, DirectX::Colors::SkyBlue, static_cast<UINT>(size(Rects)), data(Rects));
				DCL->ClearDepthStencilView(HandleDSV[0], D3D12_CLEAR_FLAG_DEPTH, 1.0f, 0, static_cast<UINT>(size(Rects)), data(Rects));

				const std::array CHs = { SwapChainBackBuffers[i].Handle };
				DCL->OMSetRenderTargets(static_cast<UINT>(size(CHs)), data(CHs), FALSE, &HandleDSV[0]);

				{
					const auto& Desc = CbvSrvUavDescs[i];
					const auto& Heap = Desc.first;
					const auto& Handle = Desc.second;

					const std::array DHs = { COM_PTR_GET(Heap) };
					DCL->SetDescriptorHeaps(static_cast<UINT>(size(DHs)), data(DHs));

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
			for (auto i = 0; i < size(Scene->RigidBodies); ++i) {
				if (i < _countof(WorldBuffer.Instances)) {
					const auto Rb = Scene->RigidBodies[i];
					if (Rb->Shape->GetShapeTyoe() == Physics::Shape::SHAPE::SPHERE) {
						const auto Pos = DirectX::XMLoadFloat4(reinterpret_cast<const DirectX::XMFLOAT4*>(static_cast<const float*>(Rb->Position)));
						const auto Rot = DirectX::XMLoadFloat4(reinterpret_cast<const DirectX::XMFLOAT4*>(static_cast<const float*>(Rb->Rotation)));
						const auto Scl = static_cast<const Physics::ShapeSphere*>(Rb->Shape)->Radius;

						DirectX::XMStoreFloat4x4(&WorldBuffer.Instances[i].World, DirectX::XMMatrixScaling(Scl, Scl, Scl) * DirectX::XMMatrixRotationQuaternion(Rot) * DirectX::XMMatrixTranslationFromVector(Pos));
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
	std::vector<UINT32> Indices;
	std::vector<DirectX::XMFLOAT3> Vertices;
	std::vector<DirectX::XMFLOAT3> Normals;

	Physics::Scene* Scene = nullptr;

	struct INSTANCE {
		DirectX::XMFLOAT4X4 World;
	};
	struct WORLD_BUFFER {
		INSTANCE Instances[64];
	};
	WORLD_BUFFER WorldBuffer;
	struct VIEW_PROJECTION_BUFFER {
		DirectX::XMFLOAT4X4 ViewProjection;
	};
	VIEW_PROJECTION_BUFFER ViewProjectionBuffer;
};
