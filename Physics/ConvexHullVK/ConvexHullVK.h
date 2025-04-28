#pragma once

#include "resource.h"

#include <numbers>

#include "../VK.h"
#include "../GltfSDK.h"
#include "Physics.h"

#define USE_MESH
#ifdef USE_MESH
#define USE_MESH_HULL
#endif

class ConvexHullVK : public Gltf::SDK, public VK
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
								std::vector<uint16_t> Indices16(Accessor.count);
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
	
	void PlaceRigidBodies(const std::vector<Math::Vec3>& Vertices) {
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
					Rb->Position = Math::Vec3(static_cast<float>(x - n2) * Offset, Y, static_cast<float>(z - n2) * Offset);
					Rb->Rotation = Math::Quat(Math::Vec3::AxisX(), TO_RADIAN(90.0f));
				}
			}
		}
		//!< 静的オブジェクト配置
		{
			constexpr auto Y = -20.0f;

			//!< 拡大する
			std::vector<Math::Vec3> ExpandedVertices(std::size(Vertices));
			std::ranges::transform(Vertices, std::back_inserter(ExpandedVertices), [&](const auto& i) { return i * FloorScale; });

			Scene->Shapes.emplace_back(std::make_unique<Physics::ShapeConvex>(ExpandedVertices));

			auto Rb = Scene->RigidBodies.emplace_back(std::make_unique<Physics::RigidBody>(Scene->Shapes.back().get(), 0.0f)).get();
			Rb->Position = Math::Vec3::AxisY() * Y;
			Rb->Rotation = Math::Quat(Math::Vec3::AxisX(), TO_RADIAN(270.0f));
			Rb->Elasticity = 0.99f;
		}
	}

	virtual void OnCreate(HWND hWnd, HINSTANCE hInstance, LPCWSTR Title) override {
#ifdef _DEBUG
		Collision::SignedVolumeTest();
#endif
		Scene = new Physics::Scene();

		VK::OnCreate(hWnd, hInstance, Title);		
	}
	virtual void OnDestroy(HWND hWnd, HINSTANCE hInstance) override {
		VK::OnDestroy(hWnd, hInstance);

		if (nullptr != Scene) {
			delete Scene;
		}
	}
	virtual void OnTimer(HWND hWnd, HINSTANCE hInstance) override {
		VK::OnTimer(hWnd, hInstance);

		if (IsUpdate()) {
			if (nullptr != Scene) {
				Scene->Update(1.0f / 60.0f);
			}
		}
	}

	virtual void DrawFrame(const UINT i) override {
		UpdateWorldBuffer();
		CopyToHostVisibleDeviceMemory(Device, UniformBuffers[i].DeviceMemory, 0, sizeof(WorldBuffer), &WorldBuffer);
	}

	virtual void AllocateCommandBuffer() override {
		VK::AllocateCommandBuffer();
		VK::AllocateSecondaryCommandBuffer(size(SwapchainBackBuffers));
	}

	virtual void CreateGeometry() override {
		std::vector<Math::Vec3> Vec3s;
#ifdef USE_MESH
		Meshes.emplace_back();
		Load(GLTF_PATH / "SuzanneMorphSparse" / "glTF-Binary" / "SuzanneMorphSparse.glb");
		Vec3s.reserve(size(Meshes.back().Vertices));
		for (auto& i : Meshes.back().Vertices) { Vec3s.emplace_back(Math::Vec3({i.x, i.y, i.z})); }
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
				Vertices_CH.emplace_back(glm::vec3(i.X(), i.Y(), i.Z()));
			}
			for (auto i : Convex->Indices) {
				Indices_CH.emplace_back(i[0]);
				Indices_CH.emplace_back(i[1]);
				Indices_CH.emplace_back(i[2]);
			}
		}

		const auto& CB = CommandBuffers[0];
		const auto PDMP = CurrentPhysicalDeviceMemoryProperties;

		Meshes.emplace_back();
		Load(ASSET_PATH / "Box.glb");
	
#ifdef USE_MESH
		const auto& Mesh = Meshes[0];
		VertexBuffers.emplace_back().Create(Device, PDMP, TotalSizeOf(Mesh.Vertices));
		VK::Scoped<StagingBuffer> StagingVertex(Device);
		StagingVertex.Create(Device, PDMP, TotalSizeOf(Mesh.Vertices), std::data(Mesh.Vertices));
		VertexBuffers.emplace_back().Create(Device, PDMP, TotalSizeOf(Mesh.Normals));
		VK::Scoped<StagingBuffer> StagingNormal(Device);
		StagingNormal.Create(Device, PDMP, TotalSizeOf(Mesh.Normals), std::data(Mesh.Normals));
		IndexBuffers.emplace_back().Create(Device, PDMP, TotalSizeOf(Mesh.Indices));
		VK::Scoped<StagingBuffer> StagingIndex(Device);
		StagingIndex.Create(Device, PDMP, TotalSizeOf(Mesh.Indices), std::data(Mesh.Indices));
		const VkDrawIndexedIndirectCommand DIIC = { 
			.indexCount = static_cast<uint32_t>(size(Mesh.Indices)),
			.instanceCount = _countof(WorldBuffer.Instances0),
			.firstIndex = 0, 
			.vertexOffset = 0, 
			.firstInstance = 0 
		};
		IndirectBuffers.emplace_back().Create(Device, PDMP, DIIC);
		VK::Scoped<StagingBuffer> StagingIndirect(Device);
		StagingIndirect.Create(Device, PDMP, sizeof(DIIC), &DIIC);
#endif

		VertexBuffers.emplace_back().Create(Device, PDMP, TotalSizeOf(Vertices_CH));
		VK::Scoped<StagingBuffer> StagingVertex_CH(Device);
		StagingVertex_CH.Create(Device, PDMP, TotalSizeOf(Vertices_CH), std::data(Vertices_CH));
		IndexBuffers.emplace_back().Create(Device, PDMP, TotalSizeOf(Indices_CH));
		VK::Scoped<StagingBuffer> StagingIndex_CH(Device);
		StagingIndex_CH.Create(Device, PDMP, TotalSizeOf(Indices_CH), std::data(Indices_CH));
		const VkDrawIndexedIndirectCommand DIIC_CH = { 
			.indexCount = static_cast<uint32_t>(size(Indices_CH)), 
			.instanceCount = _countof(WorldBuffer.Instances0),
			.firstIndex = 0, 
			.vertexOffset = 0, 
			.firstInstance = 0
		};
		IndirectBuffers.emplace_back().Create(Device, PDMP, DIIC_CH);
		VK::Scoped<StagingBuffer> StagingIndirect_CH(Device);
		StagingIndirect_CH.Create(Device, PDMP, sizeof(DIIC_CH), &DIIC_CH);

#ifdef USE_MESH
		const auto& Floor = Meshes[1];
#else
		const auto& Floor = Meshes[0];
#endif
		VertexBuffers.emplace_back().Create(Device, PDMP, TotalSizeOf(Floor.Vertices));
		VK::Scoped<StagingBuffer> StagingVertex_FLR(Device);
		StagingVertex_FLR.Create(Device, PDMP, TotalSizeOf(Floor.Vertices), std::data(Floor.Vertices));
		VertexBuffers.emplace_back().Create(Device, PDMP, TotalSizeOf(Floor.Normals));
		VK::Scoped<StagingBuffer> StagingNormal_FLR(Device);
		StagingNormal_FLR.Create(Device, PDMP, TotalSizeOf(Floor.Normals), std::data(Floor.Normals));
		IndexBuffers.emplace_back().Create(Device, PDMP, TotalSizeOf(Floor.Indices));
		VK::Scoped<StagingBuffer> StagingIndex_FLR(Device);
		StagingIndex_FLR.Create(Device, PDMP, TotalSizeOf(Floor.Indices), std::data(Floor.Indices));
		const VkDrawIndexedIndirectCommand DIIC_FLR = {
			.indexCount = static_cast<uint32_t>(size(Floor.Indices)),
			.instanceCount = _countof(WorldBuffer.Instances1),
			.firstIndex = 0,
			.vertexOffset = 0,
			.firstInstance = 0
		};
		IndirectBuffers.emplace_back().Create(Device, PDMP, DIIC_FLR);
		VK::Scoped<StagingBuffer> StagingIndirect_FLR(Device);
		StagingIndirect_FLR.Create(Device, PDMP, sizeof(DIIC_FLR), &DIIC_FLR);

		constexpr VkCommandBufferBeginInfo CBBI = { 
			.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO, 
			.pNext = nullptr, 
			.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT,
			.pInheritanceInfo = nullptr 
		};
		VERIFY_SUCCEEDED(vkBeginCommandBuffer(CB, &CBBI)); {
#ifdef USE_MESH
			const auto& Mesh = Meshes[0];
			VertexBuffers[0].PopulateCopyCommand(CB, TotalSizeOf(Mesh.Vertices), StagingVertex.Buffer);
			VertexBuffers[1].PopulateCopyCommand(CB, TotalSizeOf(Mesh.Normals), StagingNormal.Buffer);
			IndexBuffers[0].PopulateCopyCommand(CB, TotalSizeOf(Mesh.Indices), StagingIndex.Buffer);
			IndirectBuffers[0].PopulateCopyCommand(CB, sizeof(DIIC), StagingIndirect.Buffer);

			VertexBuffers[2].PopulateCopyCommand(CB, TotalSizeOf(Vertices_CH), StagingVertex_CH.Buffer);
			IndexBuffers[1].PopulateCopyCommand(CB, TotalSizeOf(Indices_CH), StagingIndex_CH.Buffer);
			IndirectBuffers[1].PopulateCopyCommand(CB, sizeof(DIIC_CH), StagingIndirect_CH.Buffer);

			const auto& Floor = Meshes[1];
			VertexBuffers[3].PopulateCopyCommand(CB, TotalSizeOf(Floor.Vertices), StagingVertex_FLR.Buffer);
			VertexBuffers[4].PopulateCopyCommand(CB, TotalSizeOf(Floor.Normals), StagingNormal_FLR.Buffer);
			IndexBuffers[2].PopulateCopyCommand(CB, TotalSizeOf(Floor.Indices), StagingIndex_FLR.Buffer);
			IndirectBuffers[2].PopulateCopyCommand(CB, sizeof(DIIC_FLR), StagingIndirect_FLR.Buffer);
#else
			VertexBuffers[0].PopulateCopyCommand(CB, TotalSizeOf(Vertices_CH), StagingVertex_CH.Buffer);
			IndexBuffers[0].PopulateCopyCommand(CB, TotalSizeOf(Indices_CH), StagingIndex_CH.Buffer);
			IndirectBuffers[0].PopulateCopyCommand(CB, sizeof(DIIC_CH), StagingIndirect_CH.Buffer); 

			const auto& Floor = Meshes[0];
			VertexBuffers[1].PopulateCopyCommand(CB, TotalSizeOf(Floor.Vertices), StagingVertex_FLR.Buffer);
			VertexBuffers[2].PopulateCopyCommand(CB, TotalSizeOf(Floor.Normals), StagingNormal_FLR.Buffer);
			IndexBuffers[1].PopulateCopyCommand(CB, TotalSizeOf(Floor.Indices), StagingIndex_FLR.Buffer);
			IndirectBuffers[1].PopulateCopyCommand(CB, sizeof(DIIC_FLR), StagingIndirect_FLR.Buffer);
#endif
		} VERIFY_SUCCEEDED(vkEndCommandBuffer(CB));
		VK::SubmitAndWait(GraphicsQueue, CB);
	}
	virtual void CreateUniformBuffer() override {
		UpdateWorldBuffer();
		UpdateViewProjectionBuffer();

		const auto PDMP = CurrentPhysicalDeviceMemoryProperties;
		for (const auto& i : SwapchainBackBuffers) {
			UniformBuffers.emplace_back().Create(Device, PDMP, sizeof(WorldBuffer));
			CopyToHostVisibleDeviceMemory(Device, UniformBuffers.back().DeviceMemory, 0, sizeof(WorldBuffer), &WorldBuffer);
		}
		for (const auto& i : SwapchainBackBuffers) {
			UniformBuffers.emplace_back().Create(Device, PDMP, sizeof(ViewProjectionBuffer));
			CopyToHostVisibleDeviceMemory(Device, UniformBuffers.back().DeviceMemory, 0, sizeof(ViewProjectionBuffer), &ViewProjectionBuffer);
		}
	}

	virtual void CreateTexture() override {
		VK::CreateTexture_Depth();
	}
	virtual void CreatePipelineLayout() override {
		CreateDescriptorSetLayout(DescriptorSetLayouts.emplace_back(), 0, {
			VkDescriptorSetLayoutBinding({.binding = 0, .descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, .descriptorCount = 1, .stageFlags = VK_SHADER_STAGE_VERTEX_BIT, .pImmutableSamplers = nullptr }),
			VkDescriptorSetLayoutBinding({.binding = 1, .descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, .descriptorCount = 1, .stageFlags = VK_SHADER_STAGE_VERTEX_BIT, .pImmutableSamplers = nullptr }),
		});
		VK::CreatePipelineLayout(PipelineLayouts.emplace_back(), { DescriptorSetLayouts[0] }, {});
	}
	virtual void CreateRenderPass() {
		VK::CreateRenderPass_Depth();
	}
	virtual void CreatePipeline() override {
		Pipelines.emplace_back();
		Pipelines.emplace_back();
		Pipelines.emplace_back();

		const std::array SMs = {
			VK::CreateShaderModule(std::filesystem::path(".") / "ConvexHullVK.vert.spv"),
			VK::CreateShaderModule(std::filesystem::path(".") / "ConvexHullVK.frag.spv"),
		};
		const std::array PSSCIs = {
			VkPipelineShaderStageCreateInfo({.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, .pNext = nullptr, .flags = 0, .stage = VK_SHADER_STAGE_VERTEX_BIT, .module = SMs[0], .pName = "main", .pSpecializationInfo = nullptr }),
			VkPipelineShaderStageCreateInfo({.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, .pNext = nullptr, .flags = 0, .stage = VK_SHADER_STAGE_FRAGMENT_BIT, .module = SMs[1], .pName = "main", .pSpecializationInfo = nullptr }),
		};
		const std::vector VIBDs = {
			VkVertexInputBindingDescription({.binding = 0, .stride = sizeof(Meshes[0].Vertices[0]), .inputRate = VK_VERTEX_INPUT_RATE_VERTEX }),
			VkVertexInputBindingDescription({.binding = 1, .stride = sizeof(Meshes[0].Normals[0]), .inputRate = VK_VERTEX_INPUT_RATE_VERTEX }),
		};
		const std::vector VIADs = {
			VkVertexInputAttributeDescription({.location = 0, .binding = 0, .format = VK_FORMAT_R32G32B32_SFLOAT, .offset = 0 }),
			VkVertexInputAttributeDescription({.location = 1, .binding = 1, .format = VK_FORMAT_R32G32B32_SFLOAT, .offset = 0 }),
		};
		constexpr VkPipelineRasterizationStateCreateInfo PRSCI = {
			.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO,
			.pNext = nullptr,
			.flags = 0,
			.depthClampEnable = VK_FALSE,
			.rasterizerDiscardEnable = VK_FALSE,
			.polygonMode = VK_POLYGON_MODE_FILL,
			.cullMode = VK_CULL_MODE_BACK_BIT,
			.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE,
			.depthBiasEnable = VK_FALSE, .depthBiasConstantFactor = 0.0f, .depthBiasClamp = 0.0f, .depthBiasSlopeFactor = 0.0f,
			.lineWidth = 1.0f
		};
		VK::CreatePipeline_VsFs_Input(Pipelines[0], PipelineLayouts[0], RenderPasses[0], VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST, 0, PRSCI, VK_TRUE, VIBDs, VIADs, PSSCIs);

		const std::array SMs_CH = {
			VK::CreateShaderModule(std::filesystem::path(".") / "ConvexHullVK_CH.vert.spv"),
			VK::CreateShaderModule(std::filesystem::path(".") / "ConvexHullVK_CH.frag.spv"),
		};
		const std::array PSSCIs_CH = {
			VkPipelineShaderStageCreateInfo({.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, .pNext = nullptr, .flags = 0, .stage = VK_SHADER_STAGE_VERTEX_BIT, .module = SMs_CH[0], .pName = "main", .pSpecializationInfo = nullptr }),
			VkPipelineShaderStageCreateInfo({.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, .pNext = nullptr, .flags = 0, .stage = VK_SHADER_STAGE_FRAGMENT_BIT, .module = SMs_CH[1], .pName = "main", .pSpecializationInfo = nullptr }),
		};
		const std::vector VIBDs_CH = {
			VkVertexInputBindingDescription({.binding = 0, .stride = sizeof(Vertices_CH[0]), .inputRate = VK_VERTEX_INPUT_RATE_VERTEX }),
		};
		const std::vector VIADs_CH = {
			VkVertexInputAttributeDescription({.location = 0, .binding = 0, .format = VK_FORMAT_R32G32B32_SFLOAT, .offset = 0 }),
		};
		constexpr VkPipelineRasterizationStateCreateInfo PRSCI_CH = {
			.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO,
			.pNext = nullptr,
			.flags = 0,
			.depthClampEnable = VK_FALSE,
			.rasterizerDiscardEnable = VK_FALSE,
			.polygonMode = VK_POLYGON_MODE_LINE,
			.cullMode = VK_CULL_MODE_BACK_BIT,
			.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE,
			.depthBiasEnable = VK_FALSE, .depthBiasConstantFactor = 0.0f, .depthBiasClamp = 0.0f, .depthBiasSlopeFactor = 0.0f,
			.lineWidth = 1.0f
		};
		VK::CreatePipeline_VsFs_Input(Pipelines[1], PipelineLayouts[0], RenderPasses[0], VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST, 0, PRSCI_CH, VK_TRUE, VIBDs_CH, VIADs_CH, PSSCIs_CH);

		const std::array SMs_FLR = {
			VK::CreateShaderModule(std::filesystem::path(".") / "ConvexHullVK_FLR.vert.spv"),
			VK::CreateShaderModule(std::filesystem::path(".") / "ConvexHullVK.frag.spv"),
		};
		const std::array PSSCIs_FLR = {
			VkPipelineShaderStageCreateInfo({.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, .pNext = nullptr, .flags = 0, .stage = VK_SHADER_STAGE_VERTEX_BIT, .module = SMs_FLR[0], .pName = "main", .pSpecializationInfo = nullptr }),
			VkPipelineShaderStageCreateInfo({.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, .pNext = nullptr, .flags = 0, .stage = VK_SHADER_STAGE_FRAGMENT_BIT, .module = SMs_FLR[1], .pName = "main", .pSpecializationInfo = nullptr }),
		};
		VK::CreatePipeline_VsFs_Input(Pipelines[2], PipelineLayouts[0], RenderPasses[0], VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST, 0, PRSCI, VK_TRUE, VIBDs, VIADs, PSSCIs_FLR);

		for (auto& i : Threads) { i.join(); }
		Threads.clear();

		for (auto i : SMs) { vkDestroyShaderModule(Device, i, GetAllocationCallbacks()); }
		for (auto i : SMs_CH) { vkDestroyShaderModule(Device, i, GetAllocationCallbacks()); }
		for (auto i : SMs_FLR) { vkDestroyShaderModule(Device, i, GetAllocationCallbacks()); }
	}
	virtual void CreateDescriptor() override {
		const auto BackBufferCount = static_cast<uint32_t>(size(SwapchainBackBuffers));
		const auto DescCount = 2;

		const auto UB0Index = 0;
		const auto UB1Index = BackBufferCount;
		const auto DSIndex = 0;

		VK::CreateDescriptorPool(DescriptorPools.emplace_back(), 0, {
			VkDescriptorPoolSize({.type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, .descriptorCount = BackBufferCount * DescCount }),
		});

		auto DSL = DescriptorSetLayouts[0];
		auto DP = DescriptorPools[0];
		const std::array DSLs = { DSL };
		for (uint32_t i = 0; i < BackBufferCount; ++i) {
			const VkDescriptorSetAllocateInfo DSAI = {
				.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO,
				.pNext = nullptr,
				.descriptorPool = DP,
				.descriptorSetCount = static_cast<uint32_t>(std::size(DSLs)), .pSetLayouts = std::data(DSLs)
			};
			VERIFY_SUCCEEDED(vkAllocateDescriptorSets(Device, &DSAI, &DescriptorSets.emplace_back()));
		}

		struct DescriptorUpdateInfo
		{
			VkDescriptorBufferInfo DBI0[1];
			VkDescriptorBufferInfo DBI1[1];
		};
		VkDescriptorUpdateTemplate DUT;
		VK::CreateDescriptorUpdateTemplate(DUT, VK_PIPELINE_BIND_POINT_GRAPHICS, {
			VkDescriptorUpdateTemplateEntry({
				.dstBinding = 0, .dstArrayElement = 0,
				.descriptorCount = _countof(DescriptorUpdateInfo::DBI0), .descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
				.offset = offsetof(DescriptorUpdateInfo, DBI0), .stride = sizeof(DescriptorUpdateInfo)
			}),	
			VkDescriptorUpdateTemplateEntry({
				.dstBinding = 1, .dstArrayElement = 0,
				.descriptorCount = _countof(DescriptorUpdateInfo::DBI1), .descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
				.offset = offsetof(DescriptorUpdateInfo, DBI1), .stride = sizeof(DescriptorUpdateInfo)
			}),
		}, DSL);
		for (uint32_t i = 0; i < BackBufferCount; ++i) {
			const DescriptorUpdateInfo DUI = {
				VkDescriptorBufferInfo({.buffer = UniformBuffers[UB0Index + i].Buffer, .offset = 0, .range = VK_WHOLE_SIZE }),
				VkDescriptorBufferInfo({.buffer = UniformBuffers[UB1Index + i].Buffer, .offset = 0, .range = VK_WHOLE_SIZE }),
			};
			vkUpdateDescriptorSetWithTemplate(Device, DescriptorSets[DSIndex + i], DUT, &DUI);
		}
		vkDestroyDescriptorUpdateTemplate(Device, DUT, GetAllocationCallbacks());
	}
	virtual void CreateFramebuffer() override {
		VK::CreateFrameBuffer_Depth(RenderPasses[0], DepthTextures[0].View);
	}
	virtual void PopulateSecondaryCommandBuffer(const size_t i) override {
		const auto RP = RenderPasses[0];
		const auto SCB = SecondaryCommandBuffers[i];
		const auto PL0 = Pipelines[0];
		const auto PL1 = Pipelines[1];
		const auto PL2 = Pipelines[2];
		const auto PLL = PipelineLayouts[0];
		const auto DS = DescriptorSets[i];

		const VkCommandBufferInheritanceInfo CBII = {
			.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_INHERITANCE_INFO,
			.pNext = nullptr,
			.renderPass = RP,
			.subpass = 0,
			.framebuffer = VK_NULL_HANDLE,
			.occlusionQueryEnable = VK_FALSE, .queryFlags = 0,
			.pipelineStatistics = 0,
		};
		const VkCommandBufferBeginInfo SCBBI = {
			.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO,
			.pNext = nullptr,
			.flags = VK_COMMAND_BUFFER_USAGE_RENDER_PASS_CONTINUE_BIT,
			.pInheritanceInfo = &CBII
		};
		VERIFY_SUCCEEDED(vkBeginCommandBuffer(SCB, &SCBBI)); {
			vkCmdSetViewport(SCB, 0, static_cast<uint32_t>(std::size(Viewports)), std::data(Viewports));
			vkCmdSetScissor(SCB, 0, static_cast<uint32_t>(std::size(ScissorRects)), std::data(ScissorRects));

			const std::array DSs = { DS };
			const std::array<uint32_t, 0> DynamicOffsets = {};
			vkCmdBindDescriptorSets(SCB, VK_PIPELINE_BIND_POINT_GRAPHICS, PLL, 0, static_cast<uint32_t>(std::size(DSs)), std::data(DSs), static_cast<uint32_t>(std::size(DynamicOffsets)), std::data(DynamicOffsets));

			const std::array Offsets = { VkDeviceSize(0) };

#ifdef USE_MESH
			//!< メッシュ
			{
				vkCmdBindPipeline(SCB, VK_PIPELINE_BIND_POINT_GRAPHICS, PL0);
				const std::array VBs = { VertexBuffers[0].Buffer };
				const std::array NBs = { VertexBuffers[1].Buffer };
				vkCmdBindVertexBuffers(SCB, 0, static_cast<uint32_t>(std::size(VBs)), std::data(VBs), std::data(Offsets));
				vkCmdBindVertexBuffers(SCB, 1, static_cast<uint32_t>(std::size(NBs)), std::data(NBs), std::data(Offsets));
				vkCmdBindIndexBuffer(SCB, IndexBuffers[0].Buffer, 0, VK_INDEX_TYPE_UINT32);
				vkCmdDrawIndexedIndirect(SCB, IndirectBuffers[0].Buffer, 0, 1, 0);
			}
#ifdef USE_MESH_HULL
			//!< 凸包
			{
				vkCmdBindPipeline(SCB, VK_PIPELINE_BIND_POINT_GRAPHICS, PL1);
				const std::array VBs_CH = { VertexBuffers[2].Buffer };
				vkCmdBindVertexBuffers(SCB, 0, static_cast<uint32_t>(std::size(VBs_CH)), std::data(VBs_CH), std::data(Offsets));
				vkCmdBindIndexBuffer(SCB, IndexBuffers[1].Buffer, 0, VK_INDEX_TYPE_UINT32);
				vkCmdDrawIndexedIndirect(SCB, IndirectBuffers[1].Buffer, 0, 1, 0);
			}
#endif
			//!< フロア
			{
				vkCmdBindPipeline(SCB, VK_PIPELINE_BIND_POINT_GRAPHICS, PL2);
				const std::array VBs_FLR = { VertexBuffers[3].Buffer };
				const std::array NBs_FLR = { VertexBuffers[4].Buffer };
				vkCmdBindVertexBuffers(SCB, 0, static_cast<uint32_t>(std::size(VBs_FLR)), std::data(VBs_FLR), std::data(Offsets));
				vkCmdBindVertexBuffers(SCB, 1, static_cast<uint32_t>(std::size(NBs_FLR)), std::data(NBs_FLR), std::data(Offsets));
				vkCmdBindIndexBuffer(SCB, IndexBuffers[2].Buffer, 0, VK_INDEX_TYPE_UINT32);
				vkCmdDrawIndexedIndirect(SCB, IndirectBuffers[2].Buffer, 0, 1, 0);
			}
#else
			//!< 凸包
			{
				vkCmdBindPipeline(SCB, VK_PIPELINE_BIND_POINT_GRAPHICS, PL1);
				const std::array VBs_CH = { VertexBuffers[0].Buffer };
				vkCmdBindVertexBuffers(SCB, 0, static_cast<uint32_t>(std::size(VBs_CH)), std::data(VBs_CH), std::data(Offsets));
				vkCmdBindIndexBuffer(SCB, IndexBuffers[0].Buffer, 0, VK_INDEX_TYPE_UINT32);
				vkCmdDrawIndexedIndirect(SCB, IndirectBuffers[0].Buffer, 0, 1, 0);
			}
			//!< フロア
			{
				vkCmdBindPipeline(SCB, VK_PIPELINE_BIND_POINT_GRAPHICS, PL2);
				const std::array VBs_FLR = { VertexBuffers[1].Buffer };
				const std::array NBs_FLR = { VertexBuffers[2].Buffer };
				vkCmdBindVertexBuffers(SCB, 0, static_cast<uint32_t>(std::size(VBs_FLR)), std::data(VBs_FLR), std::data(Offsets));
				vkCmdBindVertexBuffers(SCB, 1, static_cast<uint32_t>(std::size(NBs_FLR)), std::data(NBs_FLR), std::data(Offsets));
				vkCmdBindIndexBuffer(SCB, IndexBuffers[1].Buffer, 0, VK_INDEX_TYPE_UINT32);
				vkCmdDrawIndexedIndirect(SCB, IndirectBuffers[1].Buffer, 0, 1, 0);
			}
#endif
		} VERIFY_SUCCEEDED(vkEndCommandBuffer(SCB));
	}
	virtual void PopulateCommandBuffer(const size_t i) override {
		const auto RP = RenderPasses[0];
		const auto FB = Framebuffers[i];
		const auto SCB = SecondaryCommandBuffers[i];
		const auto CB = CommandBuffers[i];

		constexpr VkCommandBufferBeginInfo CBBI = {
			.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO,
			.pNext = nullptr,
			.flags = 0,
			.pInheritanceInfo = nullptr
		};
		VERIFY_SUCCEEDED(vkBeginCommandBuffer(CB, &CBBI)); {
			constexpr std::array CVs = { VkClearValue({.color = Colors::SkyBlue }), VkClearValue({.depthStencil = {.depth = 1.0f, .stencil = 0 } }) };
			const VkRenderPassBeginInfo RPBI = {
				.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO,
				.pNext = nullptr,
				.renderPass = RP,
				.framebuffer = FB,
				.renderArea = VkRect2D({.offset = VkOffset2D({.x = 0, .y = 0 }), .extent = SurfaceExtent2D }),
				.clearValueCount = static_cast<uint32_t>(std::size(CVs)), .pClearValues = std::data(CVs)
			};
			vkCmdBeginRenderPass(CB, &RPBI, VK_SUBPASS_CONTENTS_SECONDARY_COMMAND_BUFFERS); {
				const std::array SCBs = { SCB };
				vkCmdExecuteCommands(CB, static_cast<uint32_t>(std::size(SCBs)), std::data(SCBs));
			} vkCmdEndRenderPass(CB);
		} VERIFY_SUCCEEDED(vkEndCommandBuffer(CB));
	}

	virtual void UpdateWorldBuffer() {
		if (nullptr != Scene) {
			for (auto i = 0, i0 = 0, i1 = 0; i < size(Scene->RigidBodies); ++i) {
				const auto Rb = Scene->RigidBodies[i].get();
				const auto Pos = glm::make_vec3(static_cast<float*>(Rb->Position));
				const auto Rot = glm::make_quat(static_cast<float*>(Rb->Rotation));
				const auto Scl = 0.0f == Rb->InvMass ? glm::scale(glm::mat4(1.0f), glm::vec3(FloorScale)) : glm::mat4(1.0f);
				if (i0 < _countof(WorldBuffer.Instances0)) {
					WorldBuffer.Instances0[i0++].World = glm::translate(glm::mat4(1.0f), Pos) * glm::mat4_cast(Rot) * Scl;
				}
			}
		}
	}
	virtual void UpdateViewProjectionBuffer() {
		const auto Pos = glm::vec3(0.0f, 15.0f, 40.0f);
		const auto Tag = glm::vec3(0.0f);
		const auto Up = glm::vec3(0.0f, 1.0f, 0.0f);
		const auto View = glm::lookAt(Pos, Tag, Up);

		constexpr auto Fov = 0.16f * std::numbers::pi_v<float>;
		const auto Aspect = static_cast<float>(Rect.right - Rect.left) / (Rect.bottom - Rect.top);
		constexpr auto ZFar = 100.0f;
		constexpr auto ZNear = ZFar * 0.0001f;
		const auto Projection = glm::perspective(Fov, Aspect, ZNear, ZFar);

		ViewProjectionBuffer.ViewProjection = Projection * View;
	}

protected:
	const float FloorScale = 30.0f;

	struct MESH {
		std::vector<uint32_t> Indices;
		std::vector<glm::vec3> Vertices;
		std::vector<glm::vec3> Normals;
	};
	std::vector<MESH> Meshes;

	std::vector<uint32_t> Indices_CH;
	std::vector<glm::vec3> Vertices_CH;

	Physics::Scene* Scene = nullptr;

	struct INSTANCE {
		glm::mat4 World;
	};
	struct WORLD_BUFFER {
		INSTANCE Instances0[64];
		INSTANCE Instances1[64];
	};
	WORLD_BUFFER WorldBuffer;
	struct VIEW_PROJECTION_BUFFER {
		glm::mat4 ViewProjection;
	};
	VIEW_PROJECTION_BUFFER ViewProjectionBuffer;
};