#pragma once

#include "resource.h"

#include <numbers>

#include "../VK.h"
#include "../GltfSDK.h"
#include "Physics.h"

class ConstraintVK : public Gltf::SDK, public VK
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
								std::memcpy(data(Mesh.Vertices), data(ResourceReader->ReadBinaryData<float>(Document, Accessor)), TotalSizeOf(Mesh.Vertices));
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
								std::memcpy(data(Mesh.Normals), data(ResourceReader->ReadBinaryData<float>(Document, Accessor)), TotalSizeOf(Mesh.Normals));
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

			static_cast<Physics::ShapeBox*>(Scene->Shapes.emplace_back(new Physics::ShapeBox(Radius)))->Init();
			static_cast<Physics::ShapeSphere*>(Scene->Shapes.emplace_back(new Physics::ShapeSphere(Radius)))->Init();

			//!< コンストレイント
			{
				constexpr auto Radius = 0.5f;

				const auto JntRootPos = Math::Vec3(0.0f, 6.0f, 0.0f);
				constexpr auto JntLen = 1.25f;
				constexpr auto JntCount = 5;

				auto RbA = Scene->RigidBodies.emplace_back(new Physics::RigidBody());
				RbA->Position = JntRootPos;
				RbA->InvMass = 0;
				RbA->Init(Scene->Shapes.back());

				for (auto i = 0; i < JntCount; ++i) {
					auto RbB = Scene->RigidBodies.emplace_back(new Physics::RigidBody());
					RbB->Position = RbA->Position + Math::Vec3::AxisX() * JntLen;
					RbB->Init(Scene->Shapes[1]);

					auto Jnt = new Physics::ConstraintDistance();
					Jnt->Init(RbA, RbB, RbA->Position);
					Scene->Constraints.emplace_back(Jnt);

					RbA = RbB;
				}
			}
		}

		//!< 静的オブジェクト配置
		//{
		//	constexpr auto Radius = 20.0f;
		//	constexpr auto Y = -Radius;

		//	static_cast<Physics::ShapeBox*>(Scene->Shapes.emplace_back(new Physics::ShapeBox(Radius)))->Init();

		//	auto Rb = Scene->RigidBodies.emplace_back(new Physics::RigidBody());
		//	Rb->Position = Math::Vec3::AxisY() * Y;
		//	Rb->InvMass = 0;
		//	Rb->Elasticity = 0.99f;
		//	Rb->Init(Scene->Shapes[0]);
		//}
	}

	virtual void OnCreate(HWND hWnd, HINSTANCE hInstance, LPCWSTR Title) override {
		VK::OnCreate(hWnd, hInstance, Title);

		Scene = new Physics::Scene();
		PlaceRigidBodies();
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
		const auto& CB = CommandBuffers[0];
		const auto PDMP = CurrentPhysicalDeviceMemoryProperties;

		Meshes.emplace_back();
		Load(ASSET_PATH / "Box.glb");
		Meshes.emplace_back();
		Load(ASSET_PATH / "Sphere.glb");

		const auto& Box = Meshes[0];
		VertexBuffers.emplace_back().Create(Device, PDMP, TotalSizeOf(Box.Vertices));
		VK::Scoped<StagingBuffer> StagingVertex_Box(Device);
		StagingVertex_Box.Create(Device, PDMP, TotalSizeOf(Box.Vertices), data(Box.Vertices));

		VertexBuffers.emplace_back().Create(Device, PDMP, TotalSizeOf(Box.Normals));
		VK::Scoped<StagingBuffer> StagingNormal_Box(Device);
		StagingNormal_Box.Create(Device, PDMP, TotalSizeOf(Box.Normals), data(Box.Normals));

		IndexBuffers.emplace_back().Create(Device, PDMP, TotalSizeOf(Box.Indices));
		VK::Scoped<StagingBuffer> StagingIndex_Box(Device);
		StagingIndex_Box.Create(Device, PDMP, TotalSizeOf(Box.Indices), data(Box.Indices));

		const VkDrawIndexedIndirectCommand DIIC_Box = {
			.indexCount = static_cast<uint32_t>(size(Box.Indices)),
			.instanceCount = _countof(WorldBuffer.Instances0),
			.firstIndex = 0,
			.vertexOffset = 0,
			.firstInstance = 0
		};
		IndirectBuffers.emplace_back().Create(Device, PDMP, DIIC_Box);
		VK::Scoped<StagingBuffer> StagingIndirect_Box(Device);
		StagingIndirect_Box.Create(Device, PDMP, sizeof(DIIC_Box), &DIIC_Box);

		const auto& Sphere = Meshes[1];
		VertexBuffers.emplace_back().Create(Device, PDMP, TotalSizeOf(Sphere.Vertices));
		VK::Scoped<StagingBuffer> StagingVertex_Sphere(Device);
		StagingVertex_Sphere.Create(Device, PDMP, TotalSizeOf(Sphere.Vertices), data(Sphere.Vertices));

		VertexBuffers.emplace_back().Create(Device, PDMP, TotalSizeOf(Sphere.Normals));
		VK::Scoped<StagingBuffer> StagingNormal_Sphere(Device);
		StagingNormal_Sphere.Create(Device, PDMP, TotalSizeOf(Sphere.Normals), data(Sphere.Normals));

		IndexBuffers.emplace_back().Create(Device, PDMP, TotalSizeOf(Sphere.Indices));
		VK::Scoped<StagingBuffer> StagingIndex_Sphere(Device);
		StagingIndex_Sphere.Create(Device, PDMP, TotalSizeOf(Sphere.Indices), data(Sphere.Indices));

		const VkDrawIndexedIndirectCommand DIIC_Sphere = {
			.indexCount = static_cast<uint32_t>(size(Sphere.Indices)),
			.instanceCount = _countof(WorldBuffer.Instances1),
			.firstIndex = 0,
			.vertexOffset = 0,
			.firstInstance = 0
		};
		IndirectBuffers.emplace_back().Create(Device, PDMP, DIIC_Sphere);
		VK::Scoped<StagingBuffer> StagingIndirect_Sphere(Device);
		StagingIndirect_Sphere.Create(Device, PDMP, sizeof(DIIC_Sphere), &DIIC_Sphere);

		constexpr VkCommandBufferBeginInfo CBBI = {
			.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO,
			.pNext = nullptr,
			.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT,
			.pInheritanceInfo = nullptr
		};
		VERIFY_SUCCEEDED(vkBeginCommandBuffer(CB, &CBBI)); {
			VertexBuffers[0].PopulateCopyCommand(CB, TotalSizeOf(Box.Vertices), StagingVertex_Box.Buffer);
			VertexBuffers[1].PopulateCopyCommand(CB, TotalSizeOf(Box.Normals), StagingNormal_Box.Buffer);
			IndexBuffers[0].PopulateCopyCommand(CB, TotalSizeOf(Box.Indices), StagingIndex_Box.Buffer);
			IndirectBuffers[0].PopulateCopyCommand(CB, sizeof(DIIC_Box), StagingIndirect_Box.Buffer);

			VertexBuffers[2].PopulateCopyCommand(CB, TotalSizeOf(Sphere.Vertices), StagingVertex_Sphere.Buffer);
			VertexBuffers[3].PopulateCopyCommand(CB, TotalSizeOf(Sphere.Normals), StagingNormal_Sphere.Buffer);
			IndexBuffers[1].PopulateCopyCommand(CB, TotalSizeOf(Sphere.Indices), StagingIndex_Sphere.Buffer);
			IndirectBuffers[1].PopulateCopyCommand(CB, sizeof(DIIC_Sphere), StagingIndirect_Sphere.Buffer);
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

		const std::array SMs = {
			VK::CreateShaderModule(std::filesystem::path(".") / "ConstraintVK.vert.spv"),
			VK::CreateShaderModule(std::filesystem::path(".") / "ConstraintVK.frag.spv"),
		};
		const std::array PSSCIs = {
			VkPipelineShaderStageCreateInfo({.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, .pNext = nullptr, .flags = 0, .stage = VK_SHADER_STAGE_VERTEX_BIT, .module = SMs[0], .pName = "main", .pSpecializationInfo = nullptr }),
			VkPipelineShaderStageCreateInfo({.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, .pNext = nullptr, .flags = 0, .stage = VK_SHADER_STAGE_FRAGMENT_BIT, .module = SMs[1], .pName = "main", .pSpecializationInfo = nullptr }),
		};
		const auto& Box = Meshes[0];
		const std::vector VIBDs = {
			VkVertexInputBindingDescription({.binding = 0, .stride = sizeof(Box.Vertices[0]), .inputRate = VK_VERTEX_INPUT_RATE_VERTEX }),
			VkVertexInputBindingDescription({.binding = 1, .stride = sizeof(Box.Normals[0]), .inputRate = VK_VERTEX_INPUT_RATE_VERTEX }),
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

		const std::array SMs_1 = {
			VK::CreateShaderModule(std::filesystem::path(".") / "ConstraintVK_1.vert.spv"),
			VK::CreateShaderModule(std::filesystem::path(".") / "ConstraintVK.frag.spv"),
		};
		const std::array PSSCIs_1 = {
			VkPipelineShaderStageCreateInfo({.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, .pNext = nullptr, .flags = 0, .stage = VK_SHADER_STAGE_VERTEX_BIT, .module = SMs_1[0], .pName = "main", .pSpecializationInfo = nullptr }),
			VkPipelineShaderStageCreateInfo({.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, .pNext = nullptr, .flags = 0, .stage = VK_SHADER_STAGE_FRAGMENT_BIT, .module = SMs_1[1], .pName = "main", .pSpecializationInfo = nullptr }),
		};
		VK::CreatePipeline_VsFs_Input(Pipelines[1], PipelineLayouts[0], RenderPasses[0], VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST, 0, PRSCI, VK_TRUE, VIBDs, VIADs, PSSCIs_1);

		for (auto& i : Threads) { i.join(); }
		Threads.clear();

		for (auto i : SMs) { vkDestroyShaderModule(Device, i, GetAllocationCallbacks()); }
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
				.descriptorSetCount = static_cast<uint32_t>(size(DSLs)), .pSetLayouts = data(DSLs)
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
		const auto PL0 = Pipelines[0], PL1 = Pipelines[1];
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
			vkCmdSetViewport(SCB, 0, static_cast<uint32_t>(size(Viewports)), data(Viewports));
			vkCmdSetScissor(SCB, 0, static_cast<uint32_t>(size(ScissorRects)), data(ScissorRects));

			const std::array DSs = { DS };
			const std::array<uint32_t, 0> DynamicOffsets = {};
			vkCmdBindDescriptorSets(SCB, VK_PIPELINE_BIND_POINT_GRAPHICS, PLL, 0, static_cast<uint32_t>(size(DSs)), data(DSs), static_cast<uint32_t>(size(DynamicOffsets)), data(DynamicOffsets));

			const std::array Offsets = { VkDeviceSize(0) };

			vkCmdBindPipeline(SCB, VK_PIPELINE_BIND_POINT_GRAPHICS, PL0);
			{
				const std::array VBs = { VertexBuffers[0].Buffer };
				const std::array NBs = { VertexBuffers[1].Buffer };
				vkCmdBindVertexBuffers(SCB, 0, static_cast<uint32_t>(size(VBs)), data(VBs), data(Offsets));
				vkCmdBindVertexBuffers(SCB, 1, static_cast<uint32_t>(size(NBs)), data(NBs), data(Offsets));
				vkCmdBindIndexBuffer(SCB, IndexBuffers[0].Buffer, 0, VK_INDEX_TYPE_UINT32);
				vkCmdDrawIndexedIndirect(SCB, IndirectBuffers[0].Buffer, 0, 1, 0);
			}

			vkCmdBindPipeline(SCB, VK_PIPELINE_BIND_POINT_GRAPHICS, PL1);
			{
				const std::array VBs = { VertexBuffers[2].Buffer };
				const std::array NBs = { VertexBuffers[3].Buffer };
				vkCmdBindVertexBuffers(SCB, 0, static_cast<uint32_t>(size(VBs)), data(VBs), data(Offsets));
				vkCmdBindVertexBuffers(SCB, 1, static_cast<uint32_t>(size(NBs)), data(NBs), data(Offsets));
				vkCmdBindIndexBuffer(SCB, IndexBuffers[1].Buffer, 0, VK_INDEX_TYPE_UINT32);
				vkCmdDrawIndexedIndirect(SCB, IndirectBuffers[1].Buffer, 0, 1, 0);
			}
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
				.clearValueCount = static_cast<uint32_t>(size(CVs)), .pClearValues = data(CVs)
			};
			vkCmdBeginRenderPass(CB, &RPBI, VK_SUBPASS_CONTENTS_SECONDARY_COMMAND_BUFFERS); {
				const std::array SCBs = { SCB };
				vkCmdExecuteCommands(CB, static_cast<uint32_t>(size(SCBs)), data(SCBs));
			} vkCmdEndRenderPass(CB);
		} VERIFY_SUCCEEDED(vkEndCommandBuffer(CB));
	}

	virtual void UpdateWorldBuffer() {
		if (nullptr != Scene) {
			for (auto i = 0, i0 = 0, i1 = 0; i < size(Scene->RigidBodies); ++i) {
				const auto Rb = Scene->RigidBodies[i];
				const auto Pos = glm::make_vec3(static_cast<float*>(Rb->Position));
				const auto Rot = glm::make_quat(static_cast<float*>(Rb->Rotation));
				if (Rb->Shape->GetShapeType() == Physics::Shape::SHAPE::BOX) {
					const auto Scl = static_cast<const Physics::ShapeBox*>(Rb->Shape)->Vertices[0].X();
					if (i0 < _countof(WorldBuffer.Instances0)) {
						WorldBuffer.Instances0[i0++].World = glm::scale(glm::translate(glm::mat4(1.0f), Pos) * glm::mat4_cast(Rot), glm::vec3(Scl));
					}
				}
				if (Rb->Shape->GetShapeType() == Physics::Shape::SHAPE::SPHERE) {
					const auto Scl = static_cast<const Physics::ShapeSphere*>(Rb->Shape)->Radius;
					if (i1 < _countof(WorldBuffer.Instances1)) {
						WorldBuffer.Instances1[i1++].World = glm::scale(glm::translate(glm::mat4(1.0f), Pos) * glm::mat4_cast(Rot), glm::vec3(Scl));
					}
				}
			}
		}
	}
	virtual void UpdateViewProjectionBuffer() {
		const auto Pos = glm::vec3(0.0f, 15.0f, 30.0f);
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
	struct MESH {
		std::vector<uint32_t> Indices;
		std::vector<glm::vec3> Vertices;
		std::vector<glm::vec3> Normals;
	};
	std::vector<MESH> Meshes;

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
