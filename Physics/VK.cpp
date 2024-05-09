#include "VK.h"

#pragma comment(lib, "vulkan-1.lib")
#define VK_PROC_ADDR(proc) PFN_vk ## proc VK::vk ## proc = VK_NULL_HANDLE;
#include "VKGlobalProcAddr.h"
#include "VKInstanceProcAddr.h"
#include "VKDeviceProcAddr.h"
#undef VK_PROC_ADDR

template<> const glm::vec3 GetMin(const glm::vec3& lhs, const glm::vec3& rhs) { return glm::vec3((std::min)(lhs.x, rhs.x), (std::min)(lhs.y, rhs.y), (std::min)(lhs.z, rhs.z)); }
template<> const glm::vec3 GetMax(const glm::vec3& lhs, const glm::vec3& rhs) { return glm::vec3((std::max)(lhs.x, rhs.x), (std::max)(lhs.y, rhs.y), (std::max)(lhs.z, rhs.z)); }
template<>
void AdjustScale(std::vector<glm::vec3>& Vertices, const float Scale)
{
	auto Max = (std::numeric_limits<glm::vec3>::min)();
	auto Min = (std::numeric_limits<glm::vec3>::max)();
	for (const auto& i : Vertices) {
		Min = GetMin(Min, i);
		Max = GetMax(Max, i);
	}
	const auto Diff = Max - Min;
	const auto Bound = (std::max)((std::max)(Diff.x, Diff.y), Diff.z);
	const auto Coef = Scale / Bound;
	std::ranges::transform(Vertices, std::begin(Vertices), [&](const glm::vec3& rhs) { return glm::vec3(rhs.x, (rhs.y - Diff.y * 0.5f), rhs.z - Min.z) * Coef; });
}

void VK::OnDestroy(HWND hWnd, HINSTANCE hInstance)
{
	for (auto i : DescriptorUpdateTemplates) {
		vkDestroyDescriptorUpdateTemplate(Device, i, GetAllocationCallbacks());
	}
	DescriptorUpdateTemplates.clear();
	//!< �ʂɊJ���ł���̂� VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT ���w�肵���ꍇ
	//if (!empty(DescriptorSets)) { vkFreeDescriptorSets(Device, DescriptorPool, static_cast<uint32_t>(size(DescriptorSets)), std::data(DescriptorSets)); }
	DescriptorSets.clear();
	//!< ���Ȃ��Ă��悢
	//for (auto i : DescriptorPools) { vkResetDescriptorPool(Device, i, 0); }
	for (auto i : DescriptorPools) {
		vkDestroyDescriptorPool(Device, i, GetAllocationCallbacks());
	}
	DescriptorPools.clear();

	for (auto i : Framebuffers) {
		vkDestroyFramebuffer(Device, i, GetAllocationCallbacks());
	}
	Framebuffers.clear();
	for (auto i : Pipelines) {
		vkDestroyPipeline(Device, i, GetAllocationCallbacks());
	}
	Pipelines.clear();
	for (auto i : RenderPasses) {
		vkDestroyRenderPass(Device, i, GetAllocationCallbacks());
	}
	RenderPasses.clear();

	for (auto i : PipelineLayouts) {
		vkDestroyPipelineLayout(Device, i, GetAllocationCallbacks());
	}
	PipelineLayouts.clear();
	for (auto i : DescriptorSetLayouts) {
		vkDestroyDescriptorSetLayout(Device, i, GetAllocationCallbacks());
	}
	DescriptorSetLayouts.clear();

	for (auto& i : DepthTextures) {
		i.Destroy(Device);
	}
	DepthTextures.clear();

	for (auto i : UniformBuffers) { 
		i.Destroy(Device);
	}
	UniformBuffers.clear();

	for (auto i : IndirectBuffers) { 
		i.Destroy(Device); 
	}
	IndirectBuffers.clear();
	for (auto i : IndexBuffers) {
		i.Destroy(Device);
	}
	IndexBuffers.clear();
	for (auto i : VertexBuffers) {
		i.Destroy(Device);
	}
	VertexBuffers.clear();

	//!< �R�}���h�v�[���j�����ɃR�}���h�o�b�t�@�͈ÖٓI�ɔj�������
	for (auto i : SecondaryCommandPools) {
		vkDestroyCommandPool(Device, i, GetAllocationCallbacks());
	}
	SecondaryCommandPools.clear();
	for (auto i : CommandPools) {
		vkDestroyCommandPool(Device, i, GetAllocationCallbacks());
	}
	CommandPools.clear();

	for (auto& i : SwapchainBackBuffers) {
		vkDestroyImageView(Device, i.ImageView, GetAllocationCallbacks());
	} 
	SwapchainBackBuffers.clear();
	if (VK_NULL_HANDLE != Swapchain) [[likely]] { 
		vkDestroySwapchainKHR(Device, Swapchain, GetAllocationCallbacks()); 
		Swapchain = VK_NULL_HANDLE;
	}

	if (VK_NULL_HANDLE != RenderFinishedSemaphore) [[likely]] {
		vkDestroySemaphore(Device, RenderFinishedSemaphore, GetAllocationCallbacks());
	}
	if (VK_NULL_HANDLE != NextImageAcquiredSemaphore) [[likely]] {
		vkDestroySemaphore(Device, NextImageAcquiredSemaphore, GetAllocationCallbacks());
	}
	if (VK_NULL_HANDLE != GraphicsFence) [[likely]] {
		vkDestroyFence(Device, GraphicsFence, GetAllocationCallbacks());
	}

	if (VK_NULL_HANDLE != Device) [[likely]] {
		vkDestroyDevice(Device, GetAllocationCallbacks());
		Device = VK_NULL_HANDLE;
	}
	if (VK_NULL_HANDLE != Surface) [[likely]] {
		vkDestroySurfaceKHR(Instance, Surface, GetAllocationCallbacks());
		Surface = VK_NULL_HANDLE;
	}
	if (VK_NULL_HANDLE != Instance) [[likely]] {
		vkDestroyInstance(Instance, GetAllocationCallbacks());
		Instance = VK_NULL_HANDLE;
	}
	if (!FreeLibrary(VulkanLibrary)) [[likely]] { VulkanLibrary = nullptr; }
}

void VK::LoadVulkanLibrary() 
{
	VulkanLibrary = LoadLibrary(TEXT("vulkan-1.dll"));

#define VK_PROC_ADDR(proc) vk ## proc = reinterpret_cast<PFN_vk ## proc>(vkGetInstanceProcAddr(nullptr, "vk" #proc)); VERIFY(nullptr != vk ## proc && #proc);
#include "VKGlobalProcAddr.h"
#undef VK_PROC_ADDR
}

void VK::CreateInstance(const std::vector<const char*>& AdditionalLayers, const std::vector<const char*>& AdditionalExtensions)
{
	constexpr auto APIVersion = VK_HEADER_VERSION_COMPLETE;
	const VkApplicationInfo AI = {
		.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO,
		.pNext = nullptr,
		.pApplicationName = "VK", .applicationVersion = APIVersion,
		.pEngineName = "VK Engine Name", .engineVersion = APIVersion,
		.apiVersion = APIVersion
	};

	//!< ���C��
	std::vector Layers = {
		"VK_LAYER_KHRONOS_validation",
		"VK_LAYER_LUNARG_monitor", //!< �^�C�g���o�[��FPS��\�� (Display FPS on titile bar)
	};
	std::ranges::copy(AdditionalLayers, std::back_inserter(Layers));

	//!< �G�N�X�e���V����
	std::vector Extensions = {
	VK_KHR_SURFACE_EXTENSION_NAME,
#ifdef VK_USE_PLATFORM_WIN32_KHR
		VK_KHR_WIN32_SURFACE_EXTENSION_NAME,
#endif
#ifdef _DEBUG
		VK_EXT_DEBUG_UTILS_EXTENSION_NAME,
		VK_EXT_VALIDATION_FEATURES_EXTENSION_NAME,
#endif
	};
	std::ranges::copy(AdditionalExtensions, std::back_inserter(Extensions));

	//!< �C���X�^���X�̍쐬
	const VkInstanceCreateInfo ICI = {
		.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO,
		.pNext = nullptr,
		.flags = 0,
		.pApplicationInfo = &AI,
		.enabledLayerCount = static_cast<uint32_t>(std::size(Layers)), .ppEnabledLayerNames = std::data(Layers),
		.enabledExtensionCount = static_cast<uint32_t>(std::size(Extensions)), .ppEnabledExtensionNames = std::data(Extensions)
	};
	VERIFY_SUCCEEDED(vkCreateInstance(&ICI, GetAllocationCallbacks(), &Instance));

#define VK_PROC_ADDR(proc) vk ## proc = reinterpret_cast<PFN_vk ## proc>(vkGetInstanceProcAddr(Instance, "vk" #proc)); VERIFY(nullptr != vk ## proc && #proc);
#include "VKInstanceProcAddr.h"
#undef VK_PROC_ADDR
}

void VK::SelectPhysicalDevice(VkInstance Inst)
{
	//!< �����f�o�C�X(GPU)�̗�
	uint32_t Count = 0;
	VERIFY_SUCCEEDED(vkEnumeratePhysicalDevices(Inst, &Count, nullptr));
	PhysicalDevices.resize(Count);
	VERIFY_SUCCEEDED(vkEnumeratePhysicalDevices(Inst, &Count, std::data(PhysicalDevices)));

	//!< �����f�o�C�X�̑I���A�����ł͍ő僁������I�����邱�Ƃɂ��� (Select physical device, here select max memory size)
	const auto Index = std::distance(begin(PhysicalDevices), std::ranges::max_element(PhysicalDevices, [](const VkPhysicalDevice& lhs, const VkPhysicalDevice& rhs) {
		std::array<VkPhysicalDeviceMemoryProperties, 2> PDMPs;
		vkGetPhysicalDeviceMemoryProperties(lhs, &PDMPs[0]);
		vkGetPhysicalDeviceMemoryProperties(rhs, &PDMPs[1]);
		return std::accumulate(&PDMPs[0].memoryHeaps[0], &PDMPs[0].memoryHeaps[PDMPs[0].memoryHeapCount], static_cast<VkDeviceSize>(0), [](VkDeviceSize Sum, const VkMemoryHeap& rhs) { return Sum + rhs.size; })
			< std::accumulate(&PDMPs[1].memoryHeaps[0], &PDMPs[1].memoryHeaps[PDMPs[1].memoryHeapCount], static_cast<VkDeviceSize>(0), [](VkDeviceSize Sum, const VkMemoryHeap& rhs) { return Sum + rhs.size; });
		}));
	CurrentPhysicalDevice = PhysicalDevices[Index];
	vkGetPhysicalDeviceMemoryProperties(CurrentPhysicalDevice, &CurrentPhysicalDeviceMemoryProperties);
}
void VK::CreateDevice(HWND hWnd, HINSTANCE hInstance, void* pNext, const std::vector<const char*>& AdditionalExtensions)
{
	const VkWin32SurfaceCreateInfoKHR SCI = {
		.sType = VK_STRUCTURE_TYPE_WIN32_SURFACE_CREATE_INFO_KHR,
		.pNext = nullptr,
		.flags = 0,
		.hinstance = hInstance,
		.hwnd = hWnd
	};
	VERIFY_SUCCEEDED(vkCreateWin32SurfaceKHR(Instance, &SCI, GetAllocationCallbacks(), &Surface));

	const auto PD = CurrentPhysicalDevice;

	std::vector<VkQueueFamilyProperties> QFPs;
	{
		uint32_t Count = 0;
		vkGetPhysicalDeviceQueueFamilyProperties(PD, &Count, nullptr);
		QFPs.resize(Count);
		vkGetPhysicalDeviceQueueFamilyProperties(PD, &Count, std::data(QFPs));

		{
			constexpr uint8_t QueueFamilyPropMax = 8;
			VERIFY(size(QFPs) <= QueueFamilyPropMax);
			//!< �@�\�����L���[�t�@�~���C���f�b�N�X�̃r�b�g�𗧂ĂĂ���
			std::bitset<QueueFamilyPropMax> GraphicsMask;
			std::bitset<QueueFamilyPropMax> PresentMask;
			//!< �ŏ��Ɍ��������A�@�\�����L���[�t�@�~���C���f�b�N�X
			GraphicsQueueFamilyIndex = UINT32_MAX;
			PresentQueueFamilyIndex = UINT32_MAX;
			for (auto i = 0; i < size(QFPs); ++i) {
				if (VK_QUEUE_GRAPHICS_BIT & QFPs[i].queueFlags) {
					GraphicsMask.set(i);
					if (UINT32_MAX == GraphicsQueueFamilyIndex) {
						GraphicsQueueFamilyIndex = i;
					}
				}
				VkBool32 b = VK_FALSE;
				VERIFY_SUCCEEDED(vkGetPhysicalDeviceSurfaceSupportKHR(PD, i, Surface, &b));
				if (b) {
					PresentMask.set(i);
					if (UINT32_MAX == PresentQueueFamilyIndex) {
						PresentQueueFamilyIndex = i;
					}
				}
				if (VK_QUEUE_COMPUTE_BIT & QFPs[i].queueFlags) {}
				if (VK_QUEUE_TRANSFER_BIT & QFPs[i].queueFlags) {}
				if (VK_QUEUE_SPARSE_BINDING_BIT & QFPs[i].queueFlags) {}
				if (VK_QUEUE_PROTECTED_BIT & QFPs[i].queueFlags) {}
			}

			//!< �L���[�t�@�~�����ł̃C���f�b�N�X�y�уv���C�I���e�B�A�����ł̓O���t�B�b�N�A�v���[���g�̕����v���C�I���e�B0.5f�Œǉ����Ă���
			std::vector<std::vector<float>> Priorites(size(QFPs));
			const uint32_t GraphicsQueueIndexInFamily = static_cast<uint32_t>(size(Priorites[GraphicsQueueFamilyIndex])); Priorites[GraphicsQueueFamilyIndex].emplace_back(0.5f);
			const uint32_t PresentQueueIndexInFamily = static_cast<uint32_t>(size(Priorites[PresentQueueFamilyIndex])); Priorites[PresentQueueFamilyIndex].emplace_back(0.5f);
		
			//!< �L���[�쐬��� (Queue create information)
			std::vector<VkDeviceQueueCreateInfo> DQCIs;
			for (size_t i = 0; i < size(Priorites); ++i) {
				if (!empty(Priorites[i])) {
					DQCIs.emplace_back(
						VkDeviceQueueCreateInfo({
							.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO,
							.pNext = nullptr,
							.flags = 0,
							.queueFamilyIndex = static_cast<uint32_t>(i),
							.queueCount = static_cast<uint32_t>(std::size(Priorites[i])), .pQueuePriorities = std::data(Priorites[i])
							})
					);
				}
			}

			std::vector Extensions = { VK_KHR_SWAPCHAIN_EXTENSION_NAME };
			std::ranges::copy(AdditionalExtensions, std::back_inserter(Extensions));

			VkPhysicalDeviceFeatures PDF;
			vkGetPhysicalDeviceFeatures(PD, &PDF);
			if (nullptr == pNext) {
				const VkDeviceCreateInfo DCI = {
					.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO,
					.pNext = nullptr,
					.flags = 0,
					.queueCreateInfoCount = static_cast<uint32_t>(std::size(DQCIs)), .pQueueCreateInfos = std::data(DQCIs),
					.enabledLayerCount = 0, .ppEnabledLayerNames = nullptr,
					.enabledExtensionCount = static_cast<uint32_t>(std::size(Extensions)), .ppEnabledExtensionNames = std::data(Extensions),
					.pEnabledFeatures = &PDF
				};
				VERIFY_SUCCEEDED(vkCreateDevice(PD, &DCI, GetAllocationCallbacks(), &Device));
			}

#define VK_PROC_ADDR(proc) vk ## proc = reinterpret_cast<PFN_vk ## proc>(vkGetDeviceProcAddr(Device, "vk" #proc)); VERIFY(nullptr != vk ## proc && #proc && #proc);
#include "VKDeviceProcAddr.h"
#undef VK_PROC_ADDR

			vkGetDeviceQueue(Device, GraphicsQueueFamilyIndex, GraphicsQueueIndexInFamily, &GraphicsQueue);
			vkGetDeviceQueue(Device, PresentQueueFamilyIndex, PresentQueueIndexInFamily, &PresentQueue);
		}
	}
}

VkSurfaceFormatKHR VK::SelectSurfaceFormat(VkPhysicalDevice PD, VkSurfaceKHR Sfc)
{
	uint32_t Count = 0;
	VERIFY_SUCCEEDED(vkGetPhysicalDeviceSurfaceFormatsKHR(PD, Sfc, &Count, nullptr));
	std::vector<VkSurfaceFormatKHR> SFs(Count);
	VERIFY_SUCCEEDED(vkGetPhysicalDeviceSurfaceFormatsKHR(PD, Sfc, &Count, std::data(SFs)));

	const auto SelectedIndex = [&]() {
		//!< �v�f�� 1 �݂̂� UNDEFINED �̏ꍇ�A�����͖����D���Ȃ��̂�I���ł��� (If there is only 1 element and which is UNDEFINED, we can choose any)
		if (1 == size(SFs) && VK_FORMAT_UNDEFINED == SFs[0].format) {
			return -1;
		}
		for (auto i = 0; i < size(SFs); ++i) {
			//!< VK_FORMAT_UNDEFINED �łȂ��ŏ��̂���
			if (VK_FORMAT_UNDEFINED != SFs[i].format) {
				return i;
			}
		}
		//!< �����ɗ��Ă͂����Ȃ�
		VERIFY(false);
		return 0;
	}();

	return -1 == SelectedIndex ? VkSurfaceFormatKHR({ VK_FORMAT_B8G8R8A8_UNORM, VK_COLOR_SPACE_SRGB_NONLINEAR_KHR }) : SFs[SelectedIndex];
}
VkPresentModeKHR VK::SelectSurfacePresentMode(VkPhysicalDevice PD, VkSurfaceKHR Sfc)
{
	uint32_t Count;
	VERIFY_SUCCEEDED(vkGetPhysicalDeviceSurfacePresentModesKHR(PD, Sfc, &Count, nullptr));
	std::vector<VkPresentModeKHR> PMs(Count);
	VERIFY_SUCCEEDED(vkGetPhysicalDeviceSurfacePresentModesKHR(PD, Sfc, &Count, std::data(PMs)));

	//!< �\�Ȃ� VK_PRESENT_MODE_MAILBOX_KHR ��I���A�����łȂ���� VK_PRESENT_MODE_FIFO_KHR ��I�� (Want to select VK_PRESENT_MODE_MAILBOX_KHR, or select VK_PRESENT_MODE_FIFO_KHR)
	const auto SelectedPresentMode = [&]() {
		for (auto i : PMs) {
			if (VK_PRESENT_MODE_MAILBOX_KHR == i) {
				//!< �\�Ȃ� MAILBOX (If possible, want to use MAILBOX)
				return i;
			}
		}
		for (auto i : PMs) {
			if (VK_PRESENT_MODE_FIFO_KHR == i) {
				//!< FIFO �� VulkanAPI ���K���T�|�[�g���� (VulkanAPI always support FIFO)
				return i;
			}
		}
		VERIFY(false);
		return PMs[0];
	}();

	return SelectedPresentMode;
}

void VK::CreateSwapchain(VkPhysicalDevice PD, VkSurfaceKHR Sfc, const uint32_t Width, const uint32_t Height, const VkImageUsageFlags AdditionalUsage) 
{
	VkSurfaceCapabilitiesKHR SC;
	VERIFY_SUCCEEDED(vkGetPhysicalDeviceSurfaceCapabilitiesKHR(PD, Sfc, &SC));

	//!< �Œ����1��������肽���A�������ő�l�ŃN�����v����(maxImageCount ��0�̏ꍇ�͏������)
	const auto ImageCount = (std::min)(SC.minImageCount + 1, 0 == SC.maxImageCount ? UINT32_MAX : SC.maxImageCount);

	//!< �T�[�t�F�X�̃t�H�[�}�b�g��I��
	const auto SurfaceFormat = SelectSurfaceFormat(PD, Sfc);
	ColorFormat = SurfaceFormat.format; //!< �J���[�t�@�[�}�b�g�͊o���Ă���

	//!< �T�[�t�F�X�̃T�C�Y��I��
	//!< currentExtent.width == 0xffffffff �̏ꍇ�̓X���b�v�`�F�C���̃T�C�Y���猈�肷�� (If 0xffffffff, surface size will be determined by the extent of a swapchain targeting the surface)
	SurfaceExtent2D = 0xffffffff != SC.currentExtent.width ? SC.currentExtent : VkExtent2D({ .width = (std::clamp)(Width, SC.maxImageExtent.width, SC.minImageExtent.width), .height = (std::clamp)(Height, SC.minImageExtent.height, SC.minImageExtent.height) });

	//!< ���C���[�A�X�e���I�����_�����O�����������ꍇ��1�ȏ�ɂȂ邪�A�����ł�1
	uint32_t ImageArrayLayers = 1;

	VERIFY(VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT & SC.supportedUsageFlags);

	//!< �O���t�B�b�N�ƃv���[���g�̃L���[�t�@�~�����قȂ�ꍇ�̓L���[�t�@�~���C���f�b�N�X�̔z�񂪕K�v�A�܂� VK_SHARING_MODE_CONCURRENT ���w�肷�邱��
	//!< (������ VK_SHARING_MODE_CONCURRENT �ɂ���ƃp�t�H�[�}���X��������ꍇ������)
	std::vector<uint32_t> QueueFamilyIndices;
	if (GraphicsQueueFamilyIndex != PresentQueueFamilyIndex) {
		QueueFamilyIndices.emplace_back(GraphicsQueueFamilyIndex);
		QueueFamilyIndices.emplace_back(PresentQueueFamilyIndex);
	}

	//!< �T�[�t�F�X����]�A���]�������邩�ǂ��� (Rotate, mirror surface or not)
	const auto SurfaceTransform = (VK_SURFACE_TRANSFORM_IDENTITY_BIT_KHR & SC.supportedTransforms) ? VK_SURFACE_TRANSFORM_IDENTITY_BIT_KHR : SC.currentTransform;

	//!< �T�[�t�F�X�̃v���[���g���[�h��I��
	const auto SurfacePresentMode = SelectSurfacePresentMode(PD, Sfc);

	//!< �����̂͌�ŊJ������̂� OldSwapchain �Ɋo���Ă��� (�Z�b�e�B���O��ύX���ăX���b�v�`�F�C�����č쐬����ꍇ���ɔ�����)
	auto OldSwapchain = Swapchain;
	const VkSwapchainCreateInfoKHR SCI = {
		.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR,
		.pNext = nullptr,
		.flags = 0,
		.surface = Sfc,
		.minImageCount = ImageCount,
		.imageFormat = SurfaceFormat.format, .imageColorSpace = SurfaceFormat.colorSpace,
		.imageExtent = SurfaceExtent2D,
		.imageArrayLayers = ImageArrayLayers,
		.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | AdditionalUsage,
		.imageSharingMode = empty(QueueFamilyIndices) ? VK_SHARING_MODE_EXCLUSIVE : VK_SHARING_MODE_CONCURRENT,
		.queueFamilyIndexCount = static_cast<uint32_t>(std::size(QueueFamilyIndices)), .pQueueFamilyIndices = std::data(QueueFamilyIndices),
		.preTransform = SurfaceTransform,
		.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR,
		.presentMode = SurfacePresentMode,
		.clipped = VK_TRUE,
		.oldSwapchain = OldSwapchain
	};
	VERIFY_SUCCEEDED(vkCreateSwapchainKHR(Device, &SCI, GetAllocationCallbacks(), &Swapchain));

	//!< (�����)�O�̂�͔j��
	if (VK_NULL_HANDLE != OldSwapchain) {
		vkDestroySwapchainKHR(Device, OldSwapchain, GetAllocationCallbacks());
	}
}
void VK::GetSwapchainImages()
{
	for (auto i : SwapchainBackBuffers) {
		vkDestroyImageView(Device, i.ImageView, GetAllocationCallbacks());
	}
	SwapchainBackBuffers.clear();

	uint32_t Count;
	VERIFY_SUCCEEDED(vkGetSwapchainImagesKHR(Device, Swapchain, &Count, nullptr));
	std::vector<VkImage> Images(Count);
	VERIFY_SUCCEEDED(vkGetSwapchainImagesKHR(Device, Swapchain, &Count, std::data(Images)));
	for (auto i : Images) {
		auto& SBB = SwapchainBackBuffers.emplace_back(SwapchainBackBuffer(i));
		const VkImageViewCreateInfo IVCI = {
			.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO,
			.pNext = nullptr,
			.flags = 0,
			.image = i,
			.viewType = VK_IMAGE_VIEW_TYPE_2D,
			.format = ColorFormat,
			.components = VkComponentMapping({.r = VK_COMPONENT_SWIZZLE_IDENTITY, .g = VK_COMPONENT_SWIZZLE_IDENTITY, .b = VK_COMPONENT_SWIZZLE_IDENTITY, .a = VK_COMPONENT_SWIZZLE_IDENTITY, }),
			.subresourceRange = VkImageSubresourceRange({.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT, .baseMipLevel = 0, .levelCount = 1, .baseArrayLayer = 0, .layerCount = 1 })
		};
		VERIFY_SUCCEEDED(vkCreateImageView(Device, &IVCI, GetAllocationCallbacks(), &SBB.ImageView));
	}
}

void VK::AllocatePrimaryCommandBuffer(const size_t Num)
{
	const VkCommandPoolCreateInfo CPCI = {
		.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO,
		.pNext = nullptr,
		.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT,
		.queueFamilyIndex = GraphicsQueueFamilyIndex
	};
	VERIFY_SUCCEEDED(vkCreateCommandPool(Device, &CPCI, GetAllocationCallbacks(), &CommandPools.emplace_back()));

	const auto CP = CommandPools[0];
	CommandBuffers.resize(Num);
	const VkCommandBufferAllocateInfo CBAI = {
		.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO,
		.pNext = nullptr,
		.commandPool = CP,
		.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY,
		.commandBufferCount = static_cast<uint32_t>(size(CommandBuffers))
	};
	VERIFY_SUCCEEDED(vkAllocateCommandBuffers(Device, &CBAI, std::data(CommandBuffers)));
}
void VK::AllocateSecondaryCommandBuffer(const size_t Num) 
{
	const VkCommandPoolCreateInfo CPCI = {
		.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO,
		.pNext = nullptr,
		.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT,
		.queueFamilyIndex = GraphicsQueueFamilyIndex
	};
	VERIFY_SUCCEEDED(vkCreateCommandPool(Device, &CPCI, GetAllocationCallbacks(), &SecondaryCommandPools.emplace_back()));

	const auto SCP = SecondaryCommandPools[0];
	SecondaryCommandBuffers.resize(Num);
	const VkCommandBufferAllocateInfo CBAI = {
		.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO,
		.pNext = nullptr,
		.commandPool = SCP,
		.level = VK_COMMAND_BUFFER_LEVEL_SECONDARY,
		.commandBufferCount = static_cast<uint32_t>(size(SecondaryCommandBuffers))
	};
	VERIFY_SUCCEEDED(vkAllocateCommandBuffers(Device, &CBAI, std::data(SecondaryCommandBuffers)));
}

void VK::CreateRenderPass(VkRenderPass& RP, const std::vector<VkAttachmentDescription>& ADs, const std::vector<VkSubpassDescription>& SDs, const std::vector<VkSubpassDependency>& Deps)
{
	const VkRenderPassCreateInfo RPCI = {
		.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO,
		.pNext = nullptr,
		.flags = 0,
		.attachmentCount = static_cast<uint32_t>(std::size(ADs)), .pAttachments = std::data(ADs),
		.subpassCount = static_cast<uint32_t>(std::size(SDs)), .pSubpasses = std::data(SDs),
		.dependencyCount = static_cast<uint32_t>(std::size(Deps)), .pDependencies = std::data(Deps)
	};
	VERIFY_SUCCEEDED(vkCreateRenderPass(Device, &RPCI, GetAllocationCallbacks(), &RP));
}

void VK::CreatePipelineVsFsTesTcsGs(VkPipeline& PL,
	const VkDevice Dev,
	const VkPipelineLayout PLL,
	const VkRenderPass RP,
	const VkPrimitiveTopology PT, const uint32_t PatchControlPoints,
	const VkPipelineRasterizationStateCreateInfo& PRSCI,
	const VkPipelineDepthStencilStateCreateInfo& PDSSCI,
	const VkPipelineShaderStageCreateInfo* VS, const VkPipelineShaderStageCreateInfo* FS, const VkPipelineShaderStageCreateInfo* TES, const VkPipelineShaderStageCreateInfo* TCS, const VkPipelineShaderStageCreateInfo* GS,
	const std::vector<VkVertexInputBindingDescription>& VIBDs, const std::vector<VkVertexInputAttributeDescription>& VIADs,
	const std::vector<VkPipelineColorBlendAttachmentState>& PCBASs)
{
	//!< �V�F�[�_�X�e�[�W (ShaderStage)
	std::vector<VkPipelineShaderStageCreateInfo> PSSCIs;
	if (nullptr != VS) { PSSCIs.emplace_back(*VS); }
	if (nullptr != FS) { PSSCIs.emplace_back(*FS); }
	if (nullptr != TES) { PSSCIs.emplace_back(*TES); }
	if (nullptr != TCS) { PSSCIs.emplace_back(*TCS); }
	if (nullptr != GS) { PSSCIs.emplace_back(*GS); }
	VERIFY(!empty(PSSCIs));

	//!< �o�[�e�b�N�X�C���v�b�g (VertexInput)
	const VkPipelineVertexInputStateCreateInfo PVISCI = {
		.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO,
		.pNext = nullptr,
		.flags = 0,
		.vertexBindingDescriptionCount = static_cast<uint32_t>(std::size(VIBDs)), .pVertexBindingDescriptions = std::data(VIBDs),
		.vertexAttributeDescriptionCount = static_cast<uint32_t>(std::size(VIADs)), .pVertexAttributeDescriptions = std::data(VIADs)
	};

	//!< DX�ł́u�g�|���W�v�Ɓu�p�b�`�R���g���[���|�C���g�v�̎w���IASetPrimitiveTopology()�̈����Ƃ��ăR�}���h���X�g�֎w�肷��AVK�Ƃ͌��\�قȂ�̂Œ���
	//!< (�u�p�b�`�R���g���[���|�C���g�v�̐��������w�肷�邩�ɂ�茈�܂�)
	//!< CommandList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLESTRIP);
	//!< CommandList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_1_CONTROL_POINT_PATCHLIST);

	//!< �C���v�b�g�A�Z���u�� (InputAssembly)
	const VkPipelineInputAssemblyStateCreateInfo PIASCI = {
		.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO,
		.pNext = nullptr,
		.flags = 0,
		.topology = PT,
		.primitiveRestartEnable = VK_FALSE
	};
	//!< WITH_ADJACENCY �n�g�p���ɂ� �f�o�C�X�t�B�[�`���[ geometryShader ���L���ł��邱��
	//assert((
	//	(PIASCI.topology != VK_PRIMITIVE_TOPOLOGY_LINE_LIST_WITH_ADJACENCY || PIASCI.topology != VK_PRIMITIVE_TOPOLOGY_LINE_STRIP_WITH_ADJACENCY || PIASCI.topology != VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST_WITH_ADJACENCY || PIASCI.topology != VK_PRIMITIVE_TOPOLOGY_TRIANGLE_STRIP_WITH_ADJACENCY)
	//	|| PDF.geometryShader) /*&& ""*/);
	//!< PATCH_LIST �g�p���ɂ� �f�o�C�X�t�B�[�`���[ tessellationShader ���L���ł��邱��
	//assert((PIASCI.topology != VK_PRIMITIVE_TOPOLOGY_PATCH_LIST || PDF.tessellationShader) && "");
	//!< �C���f�b�N�X 0xffffffff(VK_INDEX_TYPE_UINT32), 0xffff(VK_INDEX_TYPE_UINT16) ���v���~�e�B�u�̃��X�^�[�g�Ƃ���A�C���f�b�N�X�n�`��̏ꍇ(vkCmdDrawIndexed, vkCmdDrawIndexedIndirect)�̂ݗL��
	//!< LIST �n�g�p�� primitiveRestartEnable �����ł��邱��
	VERIFY((
		(PIASCI.topology != VK_PRIMITIVE_TOPOLOGY_POINT_LIST || PIASCI.topology != VK_PRIMITIVE_TOPOLOGY_LINE_LIST || PIASCI.topology != VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST || PIASCI.topology != VK_PRIMITIVE_TOPOLOGY_LINE_LIST_WITH_ADJACENCY || PIASCI.topology != VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST_WITH_ADJACENCY || PIASCI.topology != VK_PRIMITIVE_TOPOLOGY_PATCH_LIST)
		|| PIASCI.primitiveRestartEnable == VK_FALSE));

	//!< �e�Z���[�V���� (Tessellation)
	VERIFY((PT != VK_PRIMITIVE_TOPOLOGY_PATCH_LIST || PatchControlPoints != 0));
	const VkPipelineTessellationStateCreateInfo PTSCI = {
		.sType = VK_STRUCTURE_TYPE_PIPELINE_TESSELLATION_STATE_CREATE_INFO,
		.pNext = nullptr,
		.flags = 0,
		.patchControlPoints = PatchControlPoints //!< �p�b�`�R���g���[���|�C���g
	};

	//!< �r���[�|�[�g (Viewport)
	//!< VkDynamicState ���g�p���邽�߁A�����ł̓r���[�|�[�g(�V�U�[)�̌��̂ݎw�肵�Ă��� (To use VkDynamicState, specify only count of viewport(scissor) here)
	//!< ��� vkCmdSetViewport(), vkCmdSetScissor() �Ŏw�肷�� (Use vkCmdSetViewport(), vkCmdSetScissor() later)
	constexpr VkPipelineViewportStateCreateInfo PVSCI = {
		.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO,
		.pNext = nullptr,
		.flags = 0,
		.viewportCount = 1, .pViewports = nullptr,
		.scissorCount = 1, .pScissors = nullptr
	};
	//!< 2�ȏ�̃r���[�|�[�g���g�p����ɂ̓f�o�C�X�t�B�[�`���[ multiViewport ���L���ł��邱�� (If use 2 or more viewport device feature multiViewport must be enabled)
	//!< �r���[�|�[�g�̃C���f�b�N�X�̓W�I���g���V�F�[�_�Ŏw�肷�� (Viewport index is specified in geometry shader)
	//assert((PVSCI.viewportCount <= 1 || PDF.multiViewport) && "");

	//!< PRSCI
	//!< FILL�ȊO�g�p���ɂ́A�f�o�C�X�t�B�[�`���[fillModeNonSolid���L���ł��邱��
	//assert(PRSCI.polygonMode == VK_POLYGON_MODE_FILL || PDF.fillModeNonSolid && "");
	//!< 1.0f ���傫�Ȓl�ɂ́A�f�o�C�X�t�B�[�`���[widelines ���L���ł��邱��
	//assert(PRSCI.lineWidth <= 1.0f || PDF.wideLines && "");

	//!< �}���`�T���v�� (Multisample)
	constexpr VkSampleMask SM = 0xffffffff; //!< 0xffffffff ���w�肷��ꍇ�́A����� nullptr �ł��悢
	const VkPipelineMultisampleStateCreateInfo PMSCI = {
		.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO,
		.pNext = nullptr,
		.flags = 0,
		.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT,
		.sampleShadingEnable = VK_FALSE, .minSampleShading = 0.0f,
		.pSampleMask = &SM,
		.alphaToCoverageEnable = VK_FALSE, .alphaToOneEnable = VK_FALSE
	};
	//assert((PMSCI.sampleShadingEnable == VK_FALSE || PDF.sampleRateShading) && "");
	VERIFY((PMSCI.minSampleShading >= 0.0f && PMSCI.minSampleShading <= 1.0f));
	//assert((PMSCI.alphaToOneEnable == VK_FALSE || PDF.alphaToOne) && "");

	//!< �J���[�u�����h (ColorBlend)
	//!< VK_BLEND_FACTOR_SRC1 �n�����g�p����ɂ́A�f�o�C�X�t�B�[�`���[ dualSrcBlend ���L���ł��邱��
	///!< SRC�R���|�[�l���g * SRC�t�@�N�^ OP DST�R���|�[�l���g * DST�t�@�N�^
	//!< �f�o�C�X�t�B�[�`���[ independentBlend ���L���Ŗ����ꍇ�́A�z��̊e�v�f�́u���S�ɓ����l�v�ł��邱�� (If device feature independentBlend is not enabled, each array element must be exactly same)
	//if (!PDF.independentBlend) {
	//	for (auto i : PCBASs) {
	//		assert(memcmp(&i, &PCBASs[0], sizeof(PCBASs[0])) == 0 && ""); //!< �ŏ��̗v�f�͔�ׂ�K�v�������܂�������
	//	}
	//}
	const VkPipelineColorBlendStateCreateInfo PCBSCI = {
		.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO,
		.pNext = nullptr,
		.flags = 0,
		.logicOpEnable = VK_FALSE, .logicOp = VK_LOGIC_OP_COPY, //!< �u�����h���ɘ_���I�y���[�V�������s�� (�u�����h�͖����ɂȂ�) (�����^�A�^�b�`�����g�ɑ΂��Ă̂�)
		.attachmentCount = static_cast<uint32_t>(std::size(PCBASs)), .pAttachments = std::data(PCBASs),
		.blendConstants = { 1.0f, 1.0f, 1.0f, 1.0f }
	};

	//!< �_�C�i�~�b�N�X�e�[�g (DynamicState)
	constexpr std::array DSs = {
		VK_DYNAMIC_STATE_VIEWPORT,
		VK_DYNAMIC_STATE_SCISSOR,
		//VK_DYNAMIC_STATE_DEPTH_BIAS,
	};
	const VkPipelineDynamicStateCreateInfo PDSCI = {
		.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO,
		.pNext = nullptr,
		.flags = 0,
		.dynamicStateCount = static_cast<uint32_t>(std::size(DSs)), .pDynamicStates = std::data(DSs)
	};

	/**
	@brief �p�� ... ���ʕ����������ꍇ�A�e�p�C�v���C�����w�肵�č쐬����Ƃ�荂���ɍ쐬�ł���A�e�q�Ԃł̃X�C�b�`��o�C���h���L��
	(DX �� D3D12_CACHED_PIPELINE_STATE ����?)
	basePipelineHandle, basePipelineIndex �͓����Ɏg�p�ł��Ȃ��A�g�p���Ȃ��ꍇ�͂��ꂼ�� VK_NULL_HANDLE, -1 ���w�肷�邱��
	�e�ɂ� VK_PIPELINE_CREATE_ALLOW_DERIVATIVES_BIT �t���O���K�v�A�q�ɂ� VK_PIPELINE_CREATE_DERIVATIVE_BIT �t���O���K�v
	�EbasePipelineHandle ... ���ɐe�Ƃ���p�C�v���C��(�n���h��)�����݂���ꍇ�Ɏw��
	�EbasePipelineIndex ... ���z����Őe�p�C�v���C���������ɍ쐬����ꍇ�A�z����ł̐e�p�C�v���C���̓Y��(�e�̓Y���̕����Ⴂ�l�ł��邱��)
	*/
	const std::array GPCIs = {
		VkGraphicsPipelineCreateInfo({
			.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO,
			.pNext = nullptr,
#ifdef _DEBUG
			.flags = VK_PIPELINE_CREATE_DISABLE_OPTIMIZATION_BIT,
#else
			.flags = 0,
#endif
			.stageCount = static_cast<uint32_t>(std::size(PSSCIs)), .pStages = std::data(PSSCIs),
			.pVertexInputState = &PVISCI,
			.pInputAssemblyState = &PIASCI,
			.pTessellationState = &PTSCI,
			.pViewportState = &PVSCI,
			.pRasterizationState = &PRSCI,
			.pMultisampleState = &PMSCI,
			.pDepthStencilState = &PDSSCI,
			.pColorBlendState = &PCBSCI,
			.pDynamicState = &PDSCI,
			.layout = PLL,
			.renderPass = RP, .subpass = 0, //!< �����Ŏw�肷�郌���_�[�p�X�́u�݊����̂�����́v�Ȃ��
			.basePipelineHandle = VK_NULL_HANDLE, .basePipelineIndex = -1
		})
	};
	//!< VK�ł�1�R�[���ŕ����̃p�C�v���C�����쐬���邱�Ƃ��ł��邪�ADX�ɍ��킹��1�������Ȃ����Ƃɂ��Ă���
	VERIFY_SUCCEEDED(vkCreateGraphicsPipelines(Dev, VK_NULL_HANDLE, static_cast<uint32_t>(std::size(GPCIs)), std::data(GPCIs), GetAllocationCallbacks(), &PL));
}

void VK::CreateViewport(const FLOAT Width, const FLOAT Height, const FLOAT MinDepth, const FLOAT MaxDepth)
{
	Viewports = {
			VkViewport({
			//!< VK�ł̓f�t�H���g�ŁuY�����v���������A�����ɕ��̒l���w�肷��ƁuY����v�������ADX�Ɠ��l�ɂȂ� (In VK, by specifying negative height, Y become up. same as DX)
			//!< �ʏ��_�́u����v���w�肷�邪�A�����ɕ��̒l���w�肷��ꍇ�́u�����v���w�肷�邱�� (When negative height, specify left bottom as base, otherwise left up)
			.x = 0.0f, .y = Height,
			.width = Width, .height = -Height,
			.minDepth = MinDepth, .maxDepth = MaxDepth
		})
	};
	//!< offset, extent �Ŏw�� (left, top, right, bottom�Ŏw���DX�Ƃ͈قȂ�̂Œ���)
	ScissorRects = {
		VkRect2D({.offset = VkOffset2D({.x = 0, .y = 0 }), .extent = VkExtent2D({.width = static_cast<uint32_t>(Width), .height = static_cast<uint32_t>(Height) }) }),
	};
}

void VK::WaitForFence(VkDevice Device, VkFence Fence) 
{
	const std::array Fences = { Fence };
	VERIFY_SUCCEEDED(vkWaitForFences(Device, static_cast<uint32_t>(std::size(Fences)), std::data(Fences), VK_TRUE, (std::numeric_limits<uint64_t>::max)()));
	vkResetFences(Device, static_cast<uint32_t>(std::size(Fences)), std::data(Fences));
}
void VK::SubmitGraphics(const uint32_t i)
{
	const std::array WaitSems = { NextImageAcquiredSemaphore };
	const std::array WaitStages = { VkPipelineStageFlags(VK_PIPELINE_STAGE_TRANSFER_BIT) };
	//!< ���s����R�}���h�o�b�t�@
	const std::array CBs = { CommandBuffers[i], };
	//!< �������ɃV�O�i�������Z�}�t�H (RenderFinishedSemaphore) -> �����҂��Ă���v���[���g���s����
	const std::array SigSems = { RenderFinishedSemaphore };
	const std::array SIs = {
		VkSubmitInfo({
			.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO,
			.pNext = nullptr,
			.waitSemaphoreCount = static_cast<uint32_t>(std::size(WaitSems)), .pWaitSemaphores = std::data(WaitSems), .pWaitDstStageMask = std::data(WaitStages), //!< ���C���[�W���擾�ł���(�v���[���g����)�܂ŃE�G�C�g
			.commandBufferCount = static_cast<uint32_t>(std::size(CBs)), .pCommandBuffers = std::data(CBs),
			.signalSemaphoreCount = static_cast<uint32_t>(std::size(SigSems)), .pSignalSemaphores = std::data(SigSems) //!< �`�抮����ʒm����
		}),
	};
	VERIFY_SUCCEEDED(vkQueueSubmit(GraphicsQueue, static_cast<uint32_t>(std::size(SIs)), std::data(SIs), GraphicsFence));
}
void VK::Present() 
{
	const std::array WaitSems = { RenderFinishedSemaphore };
	//!< �����ɕ����̃v���[���g���\�����A1�̃X���b�v�`�F�C�������1�̂�
	const std::array Swapchains = { Swapchain };
	const std::array ImageIndices = { SwapchainImageIndex };
	VERIFY(size(Swapchains) == size(ImageIndices));

	//!< �T�u�~�b�g���Ɏw�肵���Z�}�t�H(RenderFinishedSemaphore)��҂��Ă���v���[���g���s�Ȃ���
	const VkPresentInfoKHR PresentInfo = {
		.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR,
		.pNext = nullptr,
		.waitSemaphoreCount = static_cast<uint32_t>(std::size(WaitSems)), .pWaitSemaphores = std::data(WaitSems),
		.swapchainCount = static_cast<uint32_t>(std::size(Swapchains)), .pSwapchains = std::data(Swapchains), .pImageIndices = std::data(ImageIndices),
		.pResults = nullptr
	};
	VERIFY_SUCCEEDED(vkQueuePresentKHR(PresentQueue, &PresentInfo));
}
void VK::Draw() 
{
	WaitForFence(Device, GraphicsFence);

	VERIFY_SUCCEEDED(vkAcquireNextImageKHR(Device, Swapchain, UINT64_MAX, NextImageAcquiredSemaphore, VK_NULL_HANDLE, &SwapchainImageIndex));
	DrawFrame(SwapchainImageIndex);
	SubmitGraphics(SwapchainImageIndex);

	Present();
}