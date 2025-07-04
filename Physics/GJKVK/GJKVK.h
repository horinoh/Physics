#pragma once

#include "resource.h"

#include <numbers>

#include "../VK.h"
#include "../GltfSDK.h"
#include "Physics.h"

//#define USE_MESH

class GJKVK : public Gltf::SDK, public VK
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
								std::vector<uint16_t> Indices16(Accessor.count);
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
								std::memcpy(std::data(Vertices), std::data(ResourceReader->ReadBinaryData<float>(Document, Accessor)), TotalSizeOf(Vertices));
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
								std::memcpy(std::data(Normals), std::data(ResourceReader->ReadBinaryData<float>(Document, Accessor)), TotalSizeOf(Normals));
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
	virtual void OnKeyDown(HWND hWnd, HINSTANCE hInstance, const WPARAM Param) override {
		constexpr auto Speed = 0.1f;
		constexpr auto RotSpeed = 1.0f;
		auto X = 0.0f, Y = 0.0f, Z = 0.0f;
		static auto RotX = 0.0f, RotY = 0.0f;
		switch (Param) {
		case 'W':
			Y += Speed;
			break;
		case 'A':
			X -= Speed;
			break;
		case 'S':
			Y -= Speed;
			break;
		case 'D':
			X += Speed;
			break;
		case 'Q':
			Z -= Speed;
			break;
		case 'E':
			Z += Speed;
			break;

		case VK_UP:
			RotX -= RotSpeed;
			break;
		case VK_DOWN:
			RotX += RotSpeed;
			break;
		case VK_LEFT:
			RotY -= RotSpeed;
			break;
		case VK_RIGHT:
			RotY += RotSpeed;
			break;
		default:
			break;
		}
		while (RotX < 0.0f) { RotX += 360.0f; }
		while (RotX > 360.0f) { RotX -= 360.0f; }
		while (RotY < 0.0f) { RotY += 360.0f; }
		while (RotY > 360.0f) { RotY -= 360.0f; }

		if (nullptr != Scene) {
			if (0 < std::size(Scene->RigidBodies)) {
				const auto Rb = Scene->RigidBodies[0].get();

				Rb->Position = LinAlg::Vec3(
					(std::clamp)(Rb->Position.X() + X, -5.0f, 5.0f),
					(std::clamp)(Rb->Position.Y() + Y, -5.0f, 5.0f),
					(std::clamp)(Rb->Position.Z() + Z, -5.0f, 5.0f)
				);
				Rb->Rotation = LinAlg::Quat(LinAlg::Vec3::AxisY(), LinAlg::ToRadian(RotY)) * LinAlg::Quat(LinAlg::Vec3::AxisX(), LinAlg::ToRadian(RotX));
			}
		}

		Win::OnKeyDown(hWnd, hInstance, Param);
	}

	virtual void DrawFrame(const UINT i) override {
		UpdateWorldBuffer();
		CopyToHostVisibleDeviceMemory(Device, UniformBuffers[i].DeviceMemory, 0, sizeof(WorldBuffer), &WorldBuffer);
	}

	virtual void AllocateCommandBuffer() override {
		VK::AllocateCommandBuffer();
		VK::AllocateSecondaryCommandBuffer(std::size(SwapchainBackBuffers));
	}

	virtual void CreateGeometry() override {
		std::vector<LinAlg::Vec3> Vec3s;
#ifdef USE_MESH
		Load(GLTF_PATH / "SuzanneMorphSparse" / "glTF-Binary" / "SuzanneMorphSparse.glb");
		//Load(GLTF_PATH / "Avocado" / "glTF-Binary" / "Avocado.glb");
		//Load(ASSET_PATH / "bunny4.glb");
		Vec3s.reserve(std::size(Vertices));
		for (auto& i : Vertices) { Vec3s.emplace_back(Math::Vec3({ i.x, i.y, i.z })); }
#else
		//!< ダイアモンド形状
		std::vector<LinAlg::Vec3> ShapeVert;
		Physics::CreateVertices_Diamond(ShapeVert);
		std::ranges::copy(ShapeVert, std::back_inserter(Vec3s));
#endif

		//!< 凸包を構築
		auto Convex = static_cast<Physics::ShapeConvex*>(Scene->Shapes.emplace_back(std::make_unique<Physics::ShapeConvex>(Vec3s)).get());
		//!< 描画用の頂点、インデックスを構築
		for (auto& i : Convex->Vertices) {
			Vertices_CH.emplace_back(glm::vec3(i.X(), i.Y(), i.Z()));
		}
		for (auto i : Convex->Indices) {
			Indices_CH.emplace_back(i[0]);
			Indices_CH.emplace_back(i[1]);
			Indices_CH.emplace_back(i[2]);
		}

		for (auto i = 0; i < _countof(WorldBuffer.Instances); ++i) {
			auto Rb = Scene->RigidBodies.emplace_back(std::make_unique<Physics::RigidBody>(Scene->Shapes.back().get(), 0.0f)).get();
			Rb->Position = 0 == i ? LinAlg::Vec3::AxisZ() * 5.0f : LinAlg::Vec3::Zero();
			Rb->Rotation = LinAlg::Quat::Identity();
		}

		const auto& CB = CommandBuffers[0];
		const auto PDMP = CurrentPhysicalDeviceMemoryProperties;

#ifdef USE_MESH
		VertexBuffers.emplace_back().Create(Device, PDMP, TotalSizeOf(Vertices));
		VK::Scoped<StagingBuffer> StagingVertex(Device);
		StagingVertex.Create(Device, PDMP, TotalSizeOf(Vertices), std::data(Vertices));
		VertexBuffers.emplace_back().Create(Device, PDMP, TotalSizeOf(Normals));
		VK::Scoped<StagingBuffer> StagingNormal(Device);
		StagingNormal.Create(Device, PDMP, TotalSizeOf(Normals), std::data(Normals));
		IndexBuffers.emplace_back().Create(Device, PDMP, TotalSizeOf(Indices));
		VK::Scoped<StagingBuffer> StagingIndex(Device);
		StagingIndex.Create(Device, PDMP, TotalSizeOf(Indices), std::data(Indices));
		const VkDrawIndexedIndirectCommand DIIC = {
			.indexCount = static_cast<uint32_t>(std::size(Indices)),
			.instanceCount = _countof(WorldBuffer.Instances),
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
			.indexCount = static_cast<uint32_t>(std::size(Indices_CH)),
			.instanceCount = _countof(WorldBuffer.Instances),
			.firstIndex = 0,
			.vertexOffset = 0,
			.firstInstance = 0
		};
		IndirectBuffers.emplace_back().Create(Device, PDMP, DIIC_CH);
		VK::Scoped<StagingBuffer> StagingIndirect_CH(Device);
		StagingIndirect_CH.Create(Device, PDMP, sizeof(DIIC_CH), &DIIC_CH);

		const VkDrawIndirectCommand DIC_CP = {
			.vertexCount = 1,
			.instanceCount = _countof(WorldBuffer.Instances),
			.firstVertex = 0,
			.firstInstance = 0,
		};
		IndirectBuffers.emplace_back().Create(Device, PDMP, DIC_CP);
		VK::Scoped<StagingBuffer> StagingIndirect_CP(Device);
		StagingIndirect_CP.Create(Device, PDMP, sizeof(DIC_CP), &DIC_CP);

		constexpr VkCommandBufferBeginInfo CBBI = {
			.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO,
			.pNext = nullptr,
			.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT,
			.pInheritanceInfo = nullptr
		};
		VERIFY_SUCCEEDED(vkBeginCommandBuffer(CB, &CBBI)); {
#ifdef USE_MESH
			VertexBuffers[0].PopulateCopyCommand(CB, TotalSizeOf(Vertices), StagingVertex.Buffer);
			VertexBuffers[1].PopulateCopyCommand(CB, TotalSizeOf(Normals), StagingNormal.Buffer);
			IndexBuffers[0].PopulateCopyCommand(CB, TotalSizeOf(Indices), StagingIndex.Buffer);
			IndirectBuffers[0].PopulateCopyCommand(CB, sizeof(DIIC), StagingIndirect.Buffer);

			VertexBuffers[2].PopulateCopyCommand(CB, TotalSizeOf(Vertices_CH), StagingVertex_CH.Buffer);
			IndexBuffers[1].PopulateCopyCommand(CB, TotalSizeOf(Indices_CH), StagingIndex_CH.Buffer);
			IndirectBuffers[1].PopulateCopyCommand(CB, sizeof(DIIC_CH), StagingIndirect_CH.Buffer);

			IndirectBuffers[2].PopulateCopyCommand(CB, sizeof(DIC_CP), StagingIndirect_CP.Buffer);
#else
			VertexBuffers[0].PopulateCopyCommand(CB, TotalSizeOf(Vertices_CH), StagingVertex_CH.Buffer);
			IndexBuffers[0].PopulateCopyCommand(CB, TotalSizeOf(Indices_CH), StagingIndex_CH.Buffer);
			IndirectBuffers[0].PopulateCopyCommand(CB, sizeof(DIIC_CH), StagingIndirect_CH.Buffer);

			IndirectBuffers[1].PopulateCopyCommand(CB, sizeof(DIC_CP), StagingIndirect_CP.Buffer);
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
			VK::CreateShaderModule(std::filesystem::path(".") / "GJKVK_PN.vert.spv"),
			VK::CreateShaderModule(std::filesystem::path(".") / "GJKVK_PN.frag.spv"),
		};
		const std::array PSSCIs = {
			VkPipelineShaderStageCreateInfo({.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, .pNext = nullptr, .flags = 0, .stage = VK_SHADER_STAGE_VERTEX_BIT, .module = SMs[0], .pName = "main", .pSpecializationInfo = nullptr }),
			VkPipelineShaderStageCreateInfo({.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, .pNext = nullptr, .flags = 0, .stage = VK_SHADER_STAGE_FRAGMENT_BIT, .module = SMs[1], .pName = "main", .pSpecializationInfo = nullptr }),
		};
		const std::vector VIBDs = {
			VkVertexInputBindingDescription({.binding = 0, .stride = sizeof(Vertices[0]), .inputRate = VK_VERTEX_INPUT_RATE_VERTEX }),
			VkVertexInputBindingDescription({.binding = 1, .stride = sizeof(Normals[0]), .inputRate = VK_VERTEX_INPUT_RATE_VERTEX }),
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
			VK::CreateShaderModule(std::filesystem::path(".") / "GJKVK_CH_P.vert.spv"),
			VK::CreateShaderModule(std::filesystem::path(".") / "GJKVK_CH_P.frag.spv"),
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

		const std::array SMs_CP = {
			VK::CreateShaderModule(std::filesystem::path(".") / "GJKVK_CP_P.vert.spv"),
			VK::CreateShaderModule(std::filesystem::path(".") / "GJKVK_CP_P.frag.spv"),
		};
		const std::array PSSCIs_CP = {
			VkPipelineShaderStageCreateInfo({.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, .pNext = nullptr, .flags = 0, .stage = VK_SHADER_STAGE_VERTEX_BIT, .module = SMs_CP[0], .pName = "main", .pSpecializationInfo = nullptr }),
			VkPipelineShaderStageCreateInfo({.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, .pNext = nullptr, .flags = 0, .stage = VK_SHADER_STAGE_FRAGMENT_BIT, .module = SMs_CP[1], .pName = "main", .pSpecializationInfo = nullptr }),
		};
		const std::vector VIBDs_CP = {
			VkVertexInputBindingDescription({.binding = 0, .stride = sizeof(glm::vec3), .inputRate = VK_VERTEX_INPUT_RATE_VERTEX }),
		};
		const std::vector VIADs_CP = {
			VkVertexInputAttributeDescription({.location = 0, .binding = 0, .format = VK_FORMAT_R32G32B32_SFLOAT, .offset = 0 }),
		};
		constexpr VkPipelineRasterizationStateCreateInfo PRSCI_CP = {
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
		VK::CreatePipeline_VsFs_Input(Pipelines[2], PipelineLayouts[0], RenderPasses[0], VK_PRIMITIVE_TOPOLOGY_POINT_LIST, 0, PRSCI_CP, VK_TRUE, VIBDs_CP, VIADs_CP, PSSCIs_CP);

		for (auto& i : Threads) { i.join(); }
		Threads.clear();

		for (auto i : SMs) { vkDestroyShaderModule(Device, i, GetAllocationCallbacks()); }
		for (auto i : SMs_CH) { vkDestroyShaderModule(Device, i, GetAllocationCallbacks()); }
		for (auto i : SMs_CP) { vkDestroyShaderModule(Device, i, GetAllocationCallbacks()); }
	}
	virtual void CreateDescriptor() override {
		const auto BackBufferCount = static_cast<uint32_t>(std::size(SwapchainBackBuffers));
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
			vkCmdBindDescriptorSets(SCB, VK_PIPELINE_BIND_POINT_GRAPHICS, PLL, 0, static_cast<uint32_t>(std::size(DSs)), data(DSs), static_cast<uint32_t>(std::size(DynamicOffsets)), std::data(DynamicOffsets));

			const std::array Offsets = { VkDeviceSize(0) };

#ifdef USE_MESH
			vkCmdBindPipeline(SCB, VK_PIPELINE_BIND_POINT_GRAPHICS, PL0);
			const std::array VBs = { VertexBuffers[0].Buffer };
			const std::array NBs = { VertexBuffers[1].Buffer };
			vkCmdBindVertexBuffers(SCB, 0, static_cast<uint32_t>(std::size(VBs)), std::data(VBs), std::data(Offsets));
			vkCmdBindVertexBuffers(SCB, 1, static_cast<uint32_t>(std::size(NBs)), std::data(NBs), std::data(Offsets));
			vkCmdBindIndexBuffer(SCB, IndexBuffers[0].Buffer, 0, VK_INDEX_TYPE_UINT32);
			vkCmdDrawIndexedIndirect(SCB, IndirectBuffers[0].Buffer, 0, 1, 0);

			vkCmdBindPipeline(SCB, VK_PIPELINE_BIND_POINT_GRAPHICS, PL1);
			const std::array VBs_CH = { VertexBuffers[2].Buffer };
			vkCmdBindVertexBuffers(SCB, 0, static_cast<uint32_t>(std::size(VBs_CH)), std::data(VBs_CH), std::data(Offsets));
			vkCmdBindIndexBuffer(SCB, IndexBuffers[1].Buffer, 0, VK_INDEX_TYPE_UINT32);
			vkCmdDrawIndexedIndirect(SCB, IndirectBuffers[1].Buffer, 0, 1, 0);

			vkCmdBindPipeline(SCB, VK_PIPELINE_BIND_POINT_GRAPHICS, PL2);
			vkCmdDrawIndirect(SCB, IndirectBuffers[2].Buffer, 0, 1, 0);
#else
			vkCmdBindPipeline(SCB, VK_PIPELINE_BIND_POINT_GRAPHICS, PL1);
			const std::array VBs_CH = { VertexBuffers[0].Buffer };
			vkCmdBindVertexBuffers(SCB, 0, static_cast<uint32_t>(std::size(VBs_CH)), std::data(VBs_CH), std::data(Offsets));
			vkCmdBindIndexBuffer(SCB, IndexBuffers[0].Buffer, 0, VK_INDEX_TYPE_UINT32);
			vkCmdDrawIndexedIndirect(SCB, IndirectBuffers[0].Buffer, 0, 1, 0);

			vkCmdBindPipeline(SCB, VK_PIPELINE_BIND_POINT_GRAPHICS, PL2);
			vkCmdDrawIndirect(SCB, IndirectBuffers[1].Buffer, 0, 1, 0);
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
			for (auto i = 0; i < std::size(Scene->RigidBodies); ++i) {
				if (i < _countof(WorldBuffer.Instances)) {
					const auto Rb = Scene->RigidBodies[i].get();
					const auto Pos = glm::make_vec3(static_cast<float*>(Rb->Position));
					const auto Rot = glm::make_quat(static_cast<float*>(Rb->Rotation));

					WorldBuffer.Instances[i].World = glm::translate(glm::mat4(1.0f), Pos) * glm::mat4_cast(Rot);
				}
			}
			for (auto i = 0; i < std::size(Scene->RigidBodies); ++i) {
				WorldBuffer.Instances[i].Color = { 1.0f, 1.0f, 1.0f };
				WorldBuffer.Instances[i].ClosestPoint = { 0.0f, 0.0f, 0.0f };
			}
			const auto RbA = Scene->RigidBodies[0].get();
			const auto RbB = Scene->RigidBodies[1].get();
			constexpr auto WithClosestPoint = true;
			LinAlg::Vec3 OnA, OnB;
			if (Collision::Intersection::GJK_EPA(RbA, RbB, 0.01f, WithClosestPoint, OnA, OnB)) {
				WorldBuffer.Instances[0].Color = { 1.0f, 1.0f, 0.0f };
				WorldBuffer.Instances[1].Color = { 1.0f, 1.0f, 0.0f };
			}
			else if(!WithClosestPoint){
				Collision::Closest::GJK(RbA, RbB, OnA, OnB);
			}
			WorldBuffer.Instances[0].ClosestPoint = glm::vec3(OnA.X(), OnA.Y(), OnA.Z());
			WorldBuffer.Instances[1].ClosestPoint = glm::vec3(OnB.X(), OnB.Y(), OnB.Z());
			//LOG(std::data(std::format("Closest A = {}, {}, {}\n", OnA.X(), OnA.Y(), OnA.Z())));
			//LOG(std::data(std::format("Closest B = {}, {}, {}\n", OnB.X(), OnB.Y(), OnB.Z())));
		}
	}
	virtual void UpdateViewProjectionBuffer() {
		const auto Pos = glm::vec3(0.0f, 0.25f, 10.0f);
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
	std::vector<uint32_t> Indices;
	std::vector<glm::vec3> Vertices;
	std::vector<glm::vec3> Normals;

	std::vector<uint32_t> Indices_CH;
	std::vector<glm::vec3> Vertices_CH;

	Physics::Scene* Scene = nullptr;

	struct INSTANCE {
		glm::mat4 World;
		alignas(16) glm::vec3 Color;
		alignas(16) glm::vec3 ClosestPoint;
	};
	struct WORLD_BUFFER {
		INSTANCE Instances[2];
	};
	WORLD_BUFFER WorldBuffer;
	struct VIEW_PROJECTION_BUFFER {
		glm::mat4 ViewProjection;
	};
	VIEW_PROJECTION_BUFFER ViewProjectionBuffer;
};