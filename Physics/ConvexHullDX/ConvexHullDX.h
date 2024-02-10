#pragma once

#include "resource.h"

#include "../DX.h"
#include "../GltfSDK.h"

class ConvexHullDX : public Gltf::SDK, public DX
{
public:
	virtual void Process() override {
		for (const auto& i : Document.meshes.Elements()) {
			for (const auto& j : i.primitives) {
				switch (j.mode)
				{
				case Microsoft::glTF::MeshMode::MESH_POINTS:
					Topology = D3D_PRIMITIVE_TOPOLOGY_POINTLIST;
					break;
				case Microsoft::glTF::MeshMode::MESH_LINES:
					Topology = D3D_PRIMITIVE_TOPOLOGY_LINELIST;
					break;
				case Microsoft::glTF::MeshMode::MESH_LINE_LOOP: break;
				case Microsoft::glTF::MeshMode::MESH_LINE_STRIP:
					Topology = D3D_PRIMITIVE_TOPOLOGY_LINESTRIP;
					break;
				case Microsoft::glTF::MeshMode::MESH_TRIANGLES:
					Topology = D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST;
					break;
				case Microsoft::glTF::MeshMode::MESH_TRIANGLE_STRIP:
					Topology = D3D_PRIMITIVE_TOPOLOGY_TRIANGLESTRIP;
					break;
				case Microsoft::glTF::MeshMode::MESH_TRIANGLE_FAN: break;
				default: break;
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

								AdjustScale(Vertices, 1.0f);
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

	virtual void DrawFrame(const UINT i) override {
		//UpdateWorldBuffer();
		//CopyToUploadResource(COM_PTR_GET(ConstantBuffers[i].Resource), RoundUp256(sizeof(WorldBuffer)), &WorldBuffer);
	}
	virtual void CreateCommandList() override {
		DX::CreateCommandList();
		DX::CreateBundleCommandList(size(SwapChainBackBuffers) + 1);
	}
	virtual void CreateGeometry() override {
		//Load(GLTF_PATH / "bunny.glb");
		//Load(GLTF_PATH / "bunny2.glb");
		Load(GLTF_PATH / "bunny4.glb");
		//Load(GLTF_PATH / "happy_vrip.glb");
		//Load(GLTF_PATH / "dragon.glb"); 
		//Load(GLTF_PATH / "Box.glb");
		//Load(GLTF_PATH / "Sphere.glb");

#if 0
		std::vector<Vec3> Vec3s;
		Vec3s.reserve(size(Vertices));
		for (auto& i : Vertices) { Vec3s.emplace_back(Vec3({ i.x, i.y, i.z })); }

		std::vector<Vec3> HullVertices;
		std::vector<TriangleIndices> HullIndices;
		BuildConvexHull(Vec3s, HullVertices, HullIndices);
		{
			for (auto& i : HullVertices) {
				VerticesCH.emplace_back(DirectX::XMFLOAT3(i.X(), i.Y(), i.Z()));
			}
			for (auto i : HullIndices) {
				IndicesCH.emplace_back(static_cast<UINT32>(std::get<1>(i)));
				IndicesCH.emplace_back(static_cast<UINT32>(std::get<0>(i)));
				IndicesCH.emplace_back(static_cast<UINT32>(std::get<2>(i)));
			}
		}
#endif
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
			.InstanceCount = 1, 
			.StartIndexLocation = 0,
			.BaseVertexLocation = 0,
			.StartInstanceLocation = 0
		};
		IndirectBuffers.emplace_back().Create(COM_PTR_GET(Device), DIA);
		UploadResource UploadIndirect;
		UploadIndirect.Create(COM_PTR_GET(Device), sizeof(DIA), &DIA);

#if 0
		VertexBuffers.emplace_back().Create(COM_PTR_GET(Device), TotalSizeOf(Vertices_CH), sizeof(Vertices_CH[0]));
		UploadResource UploadVertex_CH;
		UploadVertex_CH.Create(COM_PTR_GET(Device), TotalSizeOf(Vertices_CH), data(Vertices_CH));

		VertexBuffers.emplace_back().Create(COM_PTR_GET(Device), TotalSizeOf(Normals_CH), sizeof(Normals_CH[0]));
		UploadResource UploadNormal_CH;
		UploadNormal_CH.Create(COM_PTR_GET(Device), TotalSizeOf(Normals_CH), data(Normals_CH));

		IndexBuffers.emplace_back().Create(COM_PTR_GET(Device), TotalSizeOf(Indices_CH), DXGI_FORMAT_R32_UINT);
		UploadResource UploadIndex_CH;
		UploadIndex_CH.Create(COM_PTR_GET(Device), TotalSizeOf(Indices_CH), data(Indices_CH));

		const D3D12_DRAW_INDEXED_ARGUMENTS DIA_CH = {
			.IndexCountPerInstance = static_cast<UINT32>(size(Indices_CH)),
			.InstanceCount = 1,
			.StartIndexLocation = 0,
			.BaseVertexLocation = 0,
			.StartInstanceLocation = 0
		};
		IndirectBuffers.emplace_back().Create(COM_PTR_GET(Device), DIA_CH);
		UploadResource UploadIndirect_CH;
		UploadIndirect_CH.Create(COM_PTR_GET(Device), sizeof(DIA_CH), &DIA_CH);
#endif

		VERIFY_SUCCEEDED(CL->Reset(CA, nullptr)); {
			VertexBuffers[0].PopulateCopyCommand(CL, TotalSizeOf(Vertices), COM_PTR_GET(UploadVertex.Resource));
			VertexBuffers[1].PopulateCopyCommand(CL, TotalSizeOf(Normals), COM_PTR_GET(UploadNormal.Resource));
			IndexBuffers[0].PopulateCopyCommand(CL, TotalSizeOf(Indices), COM_PTR_GET(UploadIndex.Resource));
			IndirectBuffers[0].PopulateCopyCommand(CL, sizeof(DIA), COM_PTR_GET(UploadIndirect.Resource));

#if 0
			VertexBuffers[2].PopulateCopyCommand(CL, TotalSizeOf(Vertices_CH), COM_PTR_GET(UploadVertex_CH.Resource));
			VertexBuffers[3].PopulateCopyCommand(CL, TotalSizeOf(Normals_CH), COM_PTR_GET(UploadNormal_CH.Resource));
			IndexBuffers[1].PopulateCopyCommand(CL, TotalSizeOf(Indices_CH), COM_PTR_GET(UploadIndex_CH.Resource));
			IndirectBuffers[1].PopulateCopyCommand(CL, sizeof(DIA_CH), COM_PTR_GET(UploadIndirect_CH.Resource));
#endif
		} VERIFY_SUCCEEDED(CL->Close());
		DX::ExecuteAndWait(CQ, CL, COM_PTR_GET(GraphicsFence));
	}
	virtual void CreateConstantBuffer() override {
		//for (const auto& i : SwapChainBackBuffers) {
		//	ConstantBuffers.emplace_back().Create(COM_PTR_GET(Device), sizeof(WorldBuffer));
		//	CopyToUploadResource(COM_PTR_GET(ConstantBuffers.back().Resource), RoundUp256(sizeof(WorldBuffer)), &WorldBuffer);
		//}
		//for (const auto& i : SwapChainBackBuffers) {
		//	ConstantBuffers.emplace_back().Create(COM_PTR_GET(Device), sizeof(ViewProjectionBuffer));
		//	CopyToUploadResource(COM_PTR_GET(ConstantBuffers.back().Resource), RoundUp256(sizeof(ViewProjectionBuffer)), &ViewProjectionBuffer);
		//}
	}
	virtual void CreateTexture() override {
		DX::CreateTexture_Depth();
	}
	virtual void CreateRootSignature() override {
		COM_PTR<ID3DBlob> Blob;
		DX::SerializeRootSignature(Blob, {}, {}, D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT | SHADER_ROOT_ACCESS_VS);
		VERIFY_SUCCEEDED(Device->CreateRootSignature(0, Blob->GetBufferPointer(), Blob->GetBufferSize(), COM_PTR_UUIDOF_PUTVOID(RootSignatures.emplace_back())));
	}
	virtual void CreatePipelineState() override {
		PipelineStates.emplace_back();
		PipelineStates.emplace_back();

		std::vector<COM_PTR<ID3DBlob>> SBs;
		VERIFY_SUCCEEDED(D3DReadFileToBlob(data((std::filesystem::path(".") / "ConvexHullDX_PN.vs.cso").wstring()), COM_PTR_PUT(SBs.emplace_back())));
		VERIFY_SUCCEEDED(D3DReadFileToBlob(data((std::filesystem::path(".") / "ConvexHullDX_PN.ps.cso").wstring()), COM_PTR_PUT(SBs.emplace_back())));
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
		VERIFY_SUCCEEDED(D3DReadFileToBlob(data((std::filesystem::path(".") / "ConvexHullDX_PN.vs.cso").wstring()), COM_PTR_PUT(SBs_CH.emplace_back())));
		VERIFY_SUCCEEDED(D3DReadFileToBlob(data((std::filesystem::path(".") / "ConvexHullDX_PN.ps.cso").wstring()), COM_PTR_PUT(SBs_CH.emplace_back())));
		const std::array SBCs_CH = {
			D3D12_SHADER_BYTECODE({.pShaderBytecode = SBs_CH[0]->GetBufferPointer(), .BytecodeLength = SBs_CH[0]->GetBufferSize() }),
			D3D12_SHADER_BYTECODE({.pShaderBytecode = SBs_CH[1]->GetBufferPointer(), .BytecodeLength = SBs_CH[1]->GetBufferSize() }),
		};
		const std::vector IEDs_CH = {
			D3D12_INPUT_ELEMENT_DESC({.SemanticName = "POSITION", .SemanticIndex = 0, .Format = DXGI_FORMAT_R32G32B32_FLOAT, .InputSlot = 0, .AlignedByteOffset = D3D12_APPEND_ALIGNED_ELEMENT, .InputSlotClass = D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, .InstanceDataStepRate = 0 }),
			D3D12_INPUT_ELEMENT_DESC({.SemanticName = "NORMAL", .SemanticIndex = 0, .Format = DXGI_FORMAT_R32G32B32_FLOAT, .InputSlot = 1, .AlignedByteOffset = D3D12_APPEND_ALIGNED_ELEMENT, .InputSlotClass = D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, .InstanceDataStepRate = 0 }),
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

		for (auto& i : Threads) { i.join(); }
		Threads.clear();
	}
	virtual void CreateDescriptor() override {
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
	virtual void PopulateBundleCommandList(const size_t i) override {
		const auto PS0 = COM_PTR_GET(PipelineStates[0]);
		const auto PS1 = COM_PTR_GET(PipelineStates[1]);
		const auto BCL = COM_PTR_GET(BundleCommandLists[i]);
		const auto BCA = COM_PTR_GET(BundleCommandAllocators[0]);

		VERIFY_SUCCEEDED(BCL->Reset(BCA, PS0));
		{
			BCL->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

			BCL->SetPipelineState(PS0);
			const std::array VBVs = { VertexBuffers[0].View, VertexBuffers[1].View };
			BCL->IASetVertexBuffers(0, static_cast<UINT>(size(VBVs)), data(VBVs));
			BCL->IASetIndexBuffer(&IndexBuffers[0].View);
			BCL->ExecuteIndirect(COM_PTR_GET(IndirectBuffers[0].CommandSignature), 1, COM_PTR_GET(IndirectBuffers[0].Resource), 0, nullptr, 0);

#if 0
			BCL->SetPipelineState(PS1);
			const std::array VBVs_CH = { VertexBuffers[2].View, VertexBuffers[3].View };
			BCL->IASetVertexBuffers(0, static_cast<UINT>(size(VBVs_CH)), data(VBVs_CH));
			BCL->IASetIndexBuffer(&IndexBuffers[1].View);
			BCL->ExecuteIndirect(COM_PTR_GET(IndirectBuffers[1].CommandSignature), 1, COM_PTR_GET(IndirectBuffers[1].Resource), 0, nullptr, 0);
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

				DCL->ExecuteBundle(BCL);
			}
			ResourceBarrier(DCL, SCR, D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_PRESENT);
		}
		VERIFY_SUCCEEDED(DCL->Close());
	}
protected:
	D3D12_PRIMITIVE_TOPOLOGY Topology;
	std::vector<UINT32> Indices;
	std::vector<DirectX::XMFLOAT3> Vertices;
	std::vector<DirectX::XMFLOAT3> Normals;

	std::vector<UINT32> Indices_CH;
	std::vector<DirectX::XMFLOAT3> Vertices_CH;
	std::vector<DirectX::XMFLOAT3> Normals_CH;
};
