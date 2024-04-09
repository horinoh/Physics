#pragma once

#include "resource.h"

#include <numbers>

#include "../VK.h"
#include "../GltfSDK.h"
#include "Physics.h"

#define USE_MULTI_VIEW

class CollisionVK : public Gltf::SDK, public VK
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
		case 'Z':
			Z -= Speed;
			break;
		case 'X':
			Z += Speed;
			break;

		case '1':
			SetColiTypes(COLLISION_TYPE::Sphere);
			break;
		case '2':
			SetColiTypes(COLLISION_TYPE::Box);
			break;
		case '3':
			SetColiTypes(COLLISION_TYPE::Cylinder);
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

		Position = Math::Vec3((std::clamp)(Position.X() + X, -5.0f, 5.0f), (std::clamp)(Position.Y() + Y, -5.0f, 5.0f), (std::clamp)(Position.Z() + Z, -5.0f, 5.0f));
		Rotation = Math::Quat(Math::Vec3::AxisY(), TO_RADIAN(RotY)) * Math::Quat(Math::Vec3::AxisX(), TO_RADIAN(RotX));
		
		Win::OnKeyDown(hWnd, hInstance, Param);
	}

	virtual void DrawFrame(const UINT i) override {
		UpdateWorldBuffer();
		CopyToHostVisibleDeviceMemory(Device, UniformBuffers[i].DeviceMemory, 0, sizeof(WorldBuffers), &WorldBuffers[0]);
	}

	virtual void AllocateCommandBuffer() override {
		VK::AllocateCommandBuffer();
		VK::AllocateSecondaryCommandBuffer(size(SwapchainBackBuffers));
	}

	virtual void CreateGeometry() override {
		Meshes.emplace_back();
		Load(ASSET_PATH / "Sphere.glb");
		Meshes.emplace_back();
		Load(ASSET_PATH / "Box.glb");
		Meshes.emplace_back();
		Load(ASSET_PATH / "Cylinder.glb");

		const auto& CB = CommandBuffers[0];
		const auto PDMP = CurrentPhysicalDeviceMemoryProperties;

		std::vector<StagingBuffer> StagingVertices;
		std::vector<StagingBuffer> StagingIndices;
		std::vector<StagingBuffer> StagingIndirects;
		std::vector<VkDrawIndexedIndirectCommand> DIICs;
		for (const auto& i : Meshes) {
			VertexBuffers.emplace_back().Create(Device, PDMP, TotalSizeOf(i.Vertices));
			StagingVertices.emplace_back().Create(Device, PDMP, TotalSizeOf(i.Vertices), data(i.Vertices));

			IndexBuffers.emplace_back().Create(Device, PDMP, TotalSizeOf(i.Indices));
			StagingIndices.emplace_back().Create(Device, PDMP, TotalSizeOf(i.Indices), data(i.Indices));

			DIICs.emplace_back(VkDrawIndexedIndirectCommand({
				.indexCount = static_cast<uint32_t>(size(i.Indices)),
				.instanceCount = _countof(WorldBuffers[0].Instances),
				.firstIndex = 0,
				.vertexOffset = 0,
				.firstInstance = 0
			}));
			IndirectBuffers.emplace_back().Create(Device, PDMP, DIICs.back());
			StagingIndirects.emplace_back().Create(Device, PDMP, sizeof(DIICs.back()), &DIICs.back());
		}

		const VkDrawIndirectCommand DIC_CP = {
			.vertexCount = 1,
			.instanceCount = _countof(WorldBuffers[0].Instances),
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
			for (auto i = 0; i < std::size(Meshes); ++i) {
				VertexBuffers[i].PopulateCopyCommand(CB, TotalSizeOf(Meshes[i].Vertices), StagingVertices[i].Buffer);
				IndexBuffers[i].PopulateCopyCommand(CB, TotalSizeOf(Meshes[i].Indices), StagingIndices[i].Buffer);
				IndirectBuffers[i].PopulateCopyCommand(CB, sizeof(DIICs[i]), StagingIndirects[i].Buffer);
			}

			IndirectBuffers[std::size(Meshes)].PopulateCopyCommand(CB, sizeof(DIC_CP), StagingIndirect_CP.Buffer);
		} VERIFY_SUCCEEDED(vkEndCommandBuffer(CB));
		VK::SubmitAndWait(GraphicsQueue, CB);

		for (auto& i : StagingVertices) { i.Destroy(Device); }
		for (auto& i : StagingIndices) { i.Destroy(Device); }
		for (auto& i : StagingIndirects) { i.Destroy(Device); }
	}
	virtual void CreateUniformBuffer() override {
		UpdateWorldBuffer();
		UpdateViewProjectionBuffer();

		const auto PDMP = CurrentPhysicalDeviceMemoryProperties;
		for (const auto& i : SwapchainBackBuffers) {
			UniformBuffers.emplace_back().Create(Device, PDMP, sizeof(WorldBuffers));
			CopyToHostVisibleDeviceMemory(Device, UniformBuffers.back().DeviceMemory, 0, sizeof(WorldBuffers), &WorldBuffers[0]);
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
			VkDescriptorSetLayoutBinding({.binding = 0, .descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC, .descriptorCount = 1, .stageFlags = VK_SHADER_STAGE_VERTEX_BIT, .pImmutableSamplers = nullptr }),
			VkDescriptorSetLayoutBinding({.binding = 1, .descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, .descriptorCount = 1, .stageFlags = VK_SHADER_STAGE_GEOMETRY_BIT, .pImmutableSamplers = nullptr }),
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
			VK::CreateShaderModule(std::filesystem::path(".") / "CollisionVK.vert.spv"),
			VK::CreateShaderModule(std::filesystem::path(".") / "CollisionVK.frag.spv"),
			VK::CreateShaderModule(std::filesystem::path(".") / "CollisionVK.geom.spv"),
		};
		const std::array PSSCIs = {
			VkPipelineShaderStageCreateInfo({.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, .pNext = nullptr, .flags = 0, .stage = VK_SHADER_STAGE_VERTEX_BIT, .module = SMs[0], .pName = "main", .pSpecializationInfo = nullptr }),
			VkPipelineShaderStageCreateInfo({.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, .pNext = nullptr, .flags = 0, .stage = VK_SHADER_STAGE_FRAGMENT_BIT, .module = SMs[1], .pName = "main", .pSpecializationInfo = nullptr }),
			VkPipelineShaderStageCreateInfo({.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, .pNext = nullptr, .flags = 0, .stage = VK_SHADER_STAGE_GEOMETRY_BIT, .module = SMs[2], .pName = "main", .pSpecializationInfo = nullptr }),
		};
		const std::vector VIBDs = {
			VkVertexInputBindingDescription({.binding = 0, .stride = sizeof(Meshes[0].Vertices[0]), .inputRate = VK_VERTEX_INPUT_RATE_VERTEX }),
		};
		const std::vector VIADs = {
			VkVertexInputAttributeDescription({.location = 0, .binding = 0, .format = VK_FORMAT_R32G32B32_SFLOAT, .offset = 0 }),
		};
		constexpr VkPipelineRasterizationStateCreateInfo PRSCI = {
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
		VK::CreatePipeline_VsFsGs_Input(Pipelines[0], PipelineLayouts[0], RenderPasses[0], VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST, 0, PRSCI, VK_TRUE, VIBDs, VIADs, PSSCIs);

		const std::array SMs_CP = {
			VK::CreateShaderModule(std::filesystem::path(".") / "CollisionVK_CP.vert.spv"),
			VK::CreateShaderModule(std::filesystem::path(".") / "CollisionVK_CP.frag.spv"),
			VK::CreateShaderModule(std::filesystem::path(".") / "CollisionVK_CP.geom.spv"),
		};
		const std::array PSSCIs_CP = {
			VkPipelineShaderStageCreateInfo({.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, .pNext = nullptr, .flags = 0, .stage = VK_SHADER_STAGE_VERTEX_BIT, .module = SMs_CP[0], .pName = "main", .pSpecializationInfo = nullptr }),
			VkPipelineShaderStageCreateInfo({.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, .pNext = nullptr, .flags = 0, .stage = VK_SHADER_STAGE_FRAGMENT_BIT, .module = SMs_CP[1], .pName = "main", .pSpecializationInfo = nullptr }),
			VkPipelineShaderStageCreateInfo({.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, .pNext = nullptr, .flags = 0, .stage = VK_SHADER_STAGE_GEOMETRY_BIT, .module = SMs_CP[2], .pName = "main", .pSpecializationInfo = nullptr }),
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
		VK::CreatePipeline_VsFsGs_Input(Pipelines[1], PipelineLayouts[0], RenderPasses[0], VK_PRIMITIVE_TOPOLOGY_POINT_LIST, 0, PRSCI_CP, VK_TRUE, VIBDs_CP, VIADs_CP, PSSCIs_CP);

		for (auto& i : Threads) { i.join(); }
		Threads.clear();

		for (auto i : SMs) { vkDestroyShaderModule(Device, i, GetAllocationCallbacks()); }
		for (auto i : SMs_CP) { vkDestroyShaderModule(Device, i, GetAllocationCallbacks()); }
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
				.descriptorCount = _countof(DescriptorUpdateInfo::DBI0), .descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC,
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
				VkDescriptorBufferInfo({.buffer = UniformBuffers[UB0Index + i].Buffer, .offset = 0, .range = sizeof(WorldBuffers[0])}),
				VkDescriptorBufferInfo({.buffer = UniformBuffers[UB1Index + i].Buffer, .offset = 0, .range = VK_WHOLE_SIZE }),
			};
			vkUpdateDescriptorSetWithTemplate(Device, DescriptorSets[DSIndex + i], DUT, &DUI);
		}
		vkDestroyDescriptorUpdateTemplate(Device, DUT, GetAllocationCallbacks());
	}
	virtual void CreateFramebuffer() override {
		VK::CreateFrameBuffer_Depth(RenderPasses[0], DepthTextures[0].View);
	}
#ifdef USE_MULTI_VIEW
	virtual void CreateViewport(const float Width, const float Height, const float MinDepth = 0.0f, const float MaxDepth = 1.0f) override {
		const auto W = Width * 0.5f, H = Height * 0.5f;
		Viewports = {
			VkViewport({.x = 0.0f, .y = H, .width = W, .height = -H, .minDepth = MinDepth, .maxDepth = MaxDepth }),
			VkViewport({.x = W, .y = H, .width = W, .height = -H, .minDepth = MinDepth, .maxDepth = MaxDepth }),
			VkViewport({.x = 0.0f, .y = Height, .width = W, .height = -H, .minDepth = MinDepth, .maxDepth = MaxDepth }),
			VkViewport({.x = W, .y = Height, .width = W, .height = -H, .minDepth = MinDepth, .maxDepth = MaxDepth }),	
		};
		const auto Ext2D = VkExtent2D({ .width = static_cast<uint32_t>(W), .height = static_cast<uint32_t>(H) });
		ScissorRects = {
			VkRect2D({ VkOffset2D({.x = 0, .y = 0 }), Ext2D }),
			VkRect2D({ VkOffset2D({.x = static_cast<int32_t>(W), .y = 0 }), Ext2D }),
			VkRect2D({ VkOffset2D({.x = 0, .y = static_cast<int32_t>(H) }), Ext2D }),
			VkRect2D({ VkOffset2D({.x = static_cast<int32_t>(W), .y = static_cast<int32_t>(H) }), Ext2D }),
		};
	}
#endif
	virtual void PopulateSecondaryCommandBuffer(const size_t i) override {
		const auto RP = RenderPasses[0];
		const auto SCB = SecondaryCommandBuffers[i];
		const auto PL0 = Pipelines[0];
		const auto PL1 = Pipelines[1];
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

			VkMemoryRequirements MR;
			vkGetBufferMemoryRequirements(Device, UniformBuffers[0].Buffer, &MR);

			vkCmdBindPipeline(SCB, VK_PIPELINE_BIND_POINT_GRAPHICS, PL0);

			const std::array DSs = { DS };
			for (auto j = 0; j < std::size(Meshes); ++j) {
				const std::array DynamicOffsets = { static_cast<uint32_t>(RoundUp(sizeof(WorldBuffers[0]) * j, MR.alignment))};
				vkCmdBindDescriptorSets(SCB, VK_PIPELINE_BIND_POINT_GRAPHICS, PLL, 0, static_cast<uint32_t>(size(DSs)), data(DSs), static_cast<uint32_t>(size(DynamicOffsets)), data(DynamicOffsets));

				const std::array VBs = { VertexBuffers[j].Buffer };
				const std::array Offsets = { VkDeviceSize(0) };
				vkCmdBindVertexBuffers(SCB, 0, static_cast<uint32_t>(size(VBs)), data(VBs), data(Offsets));
				vkCmdBindIndexBuffer(SCB, IndexBuffers[j].Buffer, 0, VK_INDEX_TYPE_UINT32);
				vkCmdDrawIndexedIndirect(SCB, IndirectBuffers[j].Buffer, 0, 1, 0);
			}

			const std::array DynamicOffsets = { static_cast<uint32_t>(0) };
			vkCmdBindDescriptorSets(SCB, VK_PIPELINE_BIND_POINT_GRAPHICS, PLL, 0, static_cast<uint32_t>(size(DSs)), data(DSs), static_cast<uint32_t>(size(DynamicOffsets)), data(DynamicOffsets));

			vkCmdBindPipeline(SCB, VK_PIPELINE_BIND_POINT_GRAPHICS, PL1);
			vkCmdDrawIndirect(SCB, IndirectBuffers[std::size(Meshes)].Buffer, 0, 1, 0);
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
		//!< カラーと最近接点をリセット
		for (auto& i : WorldBuffers) {
			i.Instances[0].Color = i.Instances[1].Color = { 1.0f, 1.0f, 1.0f };
			i.Instances[0].ClosestPoint = i.Instances[1].ClosestPoint = { 0.0f, 10.0f, 0.0f };
		}

		//!< 操作対象の位置、回転
		const auto Pos = glm::make_vec3(static_cast<float*>(Position));
		const auto Rot = glm::make_quat(static_cast<float*>(Rotation));
		for (auto i = 0; i < std::size(WorldBuffers); ++i) {
			if (i == static_cast<uint8_t>(ColiType0)) {
				WorldBuffers[i].Instances[0].World = glm::translate(glm::mat4(1.0f), Pos) * glm::mat4_cast(Rot);
			}
			if (i == static_cast<uint8_t>(ColiType1)) {
				WorldBuffers[i].Instances[1].World = glm::mat4(1.0f);
			}
		}

		constexpr auto Rad = 1.0f;
		auto& WB0 = WorldBuffers[static_cast<uint8_t>(ColiType0)];
		auto& WB1 = WorldBuffers[static_cast<uint8_t>(ColiType1)];
		auto& WBCP = WorldBuffers[0];
		switch (ColiType0)
		{
		case CollisionVK::COLLISION_TYPE::Sphere:
			switch (ColiType1)
			{
			case CollisionVK::COLLISION_TYPE::Sphere:
				if (Collision::Intersection::SphereShpere(Rad, Rad, Position, Math::Vec3::Zero())) {
					WB0.Instances[0].Color = WB1.Instances[1].Color = { 1.0f, 1.0f, 0.0f };
				}
				else {
					const auto AB = -Position.Normalize();
					const auto OnA = AB * Rad + Position;
					const auto OnB = -AB * Rad;
					WBCP.Instances[0].ClosestPoint = glm::vec3(OnA.X(), OnA.Y(), OnA.Z());
					WBCP.Instances[1].ClosestPoint = glm::vec3(OnB.X(), OnB.Y(), OnB.Z());
				}
				break;
			case CollisionVK::COLLISION_TYPE::Box:
				break;
			case CollisionVK::COLLISION_TYPE::Cylinder:
				break;
			case CollisionVK::COLLISION_TYPE::Count:
				break;
			default:
				break;
			}
			break;
		case CollisionVK::COLLISION_TYPE::Box:
			switch (ColiType1)
			{
			case CollisionVK::COLLISION_TYPE::Sphere:
				break;
			case CollisionVK::COLLISION_TYPE::Box:
				if (Rotation.NearlyEqual(Math::Quat::Identity())) {
					if (Collision::Intersection::AABBAABB(Collision::AABB(Position - Math::Vec3::One(), Position + Math::Vec3::One()), Collision::AABB(-Math::Vec3::One(), Math::Vec3::One()))) {
						WB0.Instances[0].Color = WB1.Instances[1].Color = { 1.0f, 1.0f, 0.0f };
					}
				}
				break;
			case CollisionVK::COLLISION_TYPE::Cylinder:
				break;
			case CollisionVK::COLLISION_TYPE::Count:
				break;
			default:
				break;
			}
			break;
		case CollisionVK::COLLISION_TYPE::Cylinder:
			break;
		case CollisionVK::COLLISION_TYPE::Count:
			break;
		default:
			break;
		}
	}
	virtual void UpdateViewProjectionBuffer() {
		const auto PosFrt = glm::vec3(0.0f, 0.0f, 10.0f);
		const auto PosTop = glm::vec3(0.0f, 10.0f, 0.0f);
		const auto PosRht = glm::vec3(10.0f, 0.0f, 0.0f);
		const auto PosDia = glm::normalize(glm::vec3(1.0f, 1.0f, 0.0f)) * 10.0f;
		const auto Tag = glm::vec3(0.0f);
		const auto Up = glm::vec3(0.0f, 1.0f, 0.0f);
		const auto UpT = glm::vec3(0.0f, 0.0f, -1.0f);
		const auto ViewFrt = glm::lookAt(PosFrt, Tag, Up);
		const auto ViewTop = glm::lookAt(PosTop, Tag, UpT);
		const auto ViewRht = glm::lookAt(PosRht, Tag, Up);
		const auto ViewDia = glm::lookAt(PosDia, Tag, Up);

		constexpr auto Fov = 0.16f * std::numbers::pi_v<float>;
		const auto Aspect = static_cast<float>(Rect.right - Rect.left) / (Rect.bottom - Rect.top);
		constexpr auto ZFar = 100.0f;
		constexpr auto ZNear = ZFar * 0.0001f;
		const auto Projection = glm::perspective(Fov, Aspect, ZNear, ZFar);

		ViewProjectionBuffer.ViewProjection[0] = Projection * ViewFrt;
		ViewProjectionBuffer.ViewProjection[1] = Projection * ViewTop;
		ViewProjectionBuffer.ViewProjection[2] = Projection * ViewRht;
		ViewProjectionBuffer.ViewProjection[3] = Projection * ViewDia;
	}

protected:
	enum class COLLISION_TYPE : uint8_t {
		Sphere,
		Box,
		Cylinder,

		Count,
	};
	COLLISION_TYPE ColiType0 = COLLISION_TYPE::Sphere;
	COLLISION_TYPE ColiType1 = COLLISION_TYPE::Sphere;
	void SetColiType0(const COLLISION_TYPE Type) {
		if (ColiType0 != Type) {
			if (_countof(WorldBuffers) > static_cast<uint8_t>(ColiType0)) {
				WorldBuffers[static_cast<uint8_t>(ColiType0)].Instances[0].World = glm::scale(glm::mat4(), glm::vec3(0.0f, 0.0f, 0.0f));
			}
			ColiType0 = Type;
		}
	}
	void SetColiType1(const COLLISION_TYPE Type) {
		if (ColiType1 != Type) {
			if (_countof(WorldBuffers) > static_cast<uint8_t>(ColiType1)) {
				WorldBuffers[static_cast<uint8_t>(ColiType1)].Instances[1].World = glm::scale(glm::mat4(), glm::vec3(0.0f, 0.0f, 0.0f));
			}
			ColiType1 = Type;
		}
	}
	void SetColiTypes(const COLLISION_TYPE Type) {
		if (GetKeyState(VK_SHIFT) < 0) {
			SetColiType1(Type);
		}
		else {
			SetColiType0(Type);
		}
	}

	Math::Vec3 Position = Math::Vec3::AxisZ() * 5.0f;
	Math::Quat Rotation;

	struct MESH {
		std::vector<uint32_t> Indices;
		std::vector<glm::vec3> Vertices;
		std::vector<glm::vec3> Normals;
	};
	std::vector<MESH> Meshes;

	struct INSTANCE {
		glm::mat4 World;
		alignas(16) glm::vec3 Color;
		alignas(16) glm::vec3 ClosestPoint;
	};
	struct WORLD_BUFFER {
		INSTANCE Instances[2];
	};
	WORLD_BUFFER WorldBuffers[static_cast<uint8_t>(COLLISION_TYPE::Count)];

	struct VIEW_PROJECTION_BUFFER {
		glm::mat4 ViewProjection[4];
	};
	VIEW_PROJECTION_BUFFER ViewProjectionBuffer;
};