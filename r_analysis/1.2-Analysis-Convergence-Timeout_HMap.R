library(ggplot2)
library(tidyr)
library(dplyr)
library(extrafont)
library(emmeans)
library(brglm2)

# --- STEP 1: INITIALIZE FONTS (For Times New Roman 12pt) ---
# Run font_import(pattern = "Times", prompt = FALSE) once if you haven't before
loadfonts(device = "pdf", quiet = TRUE)
# --- STEP 2: ASSIGN EXISTING DATA FRAMES ---
DATA_DIR <- "../data" 

path_HMap_1s <- file.path(DATA_DIR, "h_map/HMap_1s.xlsx")
path_HMap_5s <- file.path(DATA_DIR, "h_map/HMap_5s.xlsx")

df_HMap_1s_raw <- read_xlsx(path_HMap_1s)
df_HMap_5s_raw <- read_xlsx(path_HMap_5s)

# Dynamically force the first column name to "Metric" regardless of its current name
colnames(df_HMap_1s_raw)[1] <- "Metric"
colnames(df_HMap_5s_raw)[1] <- "Metric"


# --- STEP 3: TRANSFORM WIDE TO LONG FORMAT ---
HMap_process_dataset <- function(df, duration_label) {
  df %>%
    # Clean string spacing/capitalization discrepancies if they exist
    mutate(Metric = trimws(tolower(Metric))) %>%
    
    # Filter out only the 5 Metric categories needed for the stacked bars
    # (Adjust strings here if your rows use slightly different naming)
    filter(Metric %in% c("true success", "false success", "collision", "timeout", "other failure")) %>%
    
    # Pivot the 7 algorithm columns into a single long column
    pivot_longer(cols = -Metric, names_to = "Algorithm", values_to = "Count") %>%
    
    # Label the duration group
    mutate(Duration = duration_label)
}

# Run the transformation pipeline
cleaned_HMap_1s <- HMap_process_dataset(df_HMap_1s_raw, "1s Duration")
cleaned_HMap_5s <- HMap_process_dataset(df_HMap_5s_raw, "5s Duration")
plot_data_HMap  <- bind_rows(cleaned_HMap_1s, cleaned_HMap_5s)

# --- STEP 4: STANDARDIZE LABELS & LAYER ORDER (UPDATED) ---
# Capture the exact column order of your algorithms from the original dataset
# (We use [-1] to skip the first column, which is the "Metric" column)
original_algo_order_HMap <- colnames(df_HMap_1s_raw)[-1]

plot_data_HMap <- plot_data_HMap %>%
  mutate(
    Metric = case_when(
      Metric == "true success"   ~ "True Success",
      Metric == "false success"  ~ "False Success",
      Metric == "collision"      ~ "Collision",
      Metric == "timeout"        ~ "Timeout",
      Metric == "other failure"  ~ "Other Failure",
      TRUE                       ~ Metric
    ),
    # 1. Enforce the exact dataset column order for the X-axis
    Algorithm = factor(Algorithm, levels = original_algo_order_HMap),
    
    # 2. Sets the visual stack ordering from bottom to top
    Metric = factor(Metric, levels = c("Other Failure", "Timeout", "Collision", "False Success", "True Success"))
  )


# ==========================================
# ==========================================
# ==========================================
# ==========================================

# Timeout Analysis
# ==========================================


library(emmeans)
library(brglm2) 



# Step 1: Filter collisions metrics and select your 5 algorithms
df_eff_hmap <- subset(plot_data_HMap, 
                      Metric %in% c("True Success", "False Success", "Timeout") & 
                        Algorithm %in% c("Standard AIC", "Deep AIC", "Full-Distribution AIC", "Purely Epistemic AIC", "Constrained Random", "D-Optimality"))

# Step 2: Code the binary dependent variable
df_eff_hmap$Is_Timeout <- ifelse(df_eff_hmap$Metric == "Timeout", 1, 0)

# --- H-MAP 1s DURATION ---
df_hmap_1s <- subset(df_eff_hmap, Duration == "1s Duration" | Duration == "1s")

model_hmap_1s <- glm(Is_Timeout ~ Algorithm, 
                     data = df_hmap_1s, 
                     weights = Count, 
                     family = binomial,
                     method = "brglmFit") # <- Handles 0 counts cleanly

pairs_hmap_1s <- emmeans(model_hmap_1s, pairwise ~ Algorithm, type = "response")
print(pairs_hmap_1s$emmeans)
print(pairs_hmap_1s$contrasts)

# --- H-MAP 5s DURATION ---
df_hmap_5s <- subset(df_eff_hmap, Duration == "5s Duration" | Duration == "5s")

model_hmap_5s <- glm(Is_Timeout ~ Algorithm, 
                     data = df_hmap_5s, 
                     weights = Count, 
                     family = binomial,
                     method = "brglmFit") # <- Handles 0 counts cleanly

pairs_hmap_5s <- emmeans(model_hmap_5s, pairwise ~ Algorithm, type = "response")
print(pairs_hmap_5s$emmeans)
print(pairs_hmap_5s$contrasts)


# ==========================================
# ==========================================
# ==========================================
# ==========================================
# ==========================================
# Precision Analysis
# ==========================================


# 1. Filter out collision runs to level the playing field
df_discounted <- subset(plot_data_HMap, 
                        Metric %in% c("True Success", "False Success", "Timeout") & 
                          Algorithm %in% c("Standard AIC", 
                                           "Deep AIC", 
                                           "Full-Distribution AIC",
                                           "Purely Epistemic AIC",
                                           "Constrained Random",
                                           "D-Optimality"))


# 2. Precision Model: Within converged runs, what are the odds of True Success?
df_converged <- subset(df_discounted, Metric %in% c("True Success", "False Success"))
df_converged$Is_True_Success <- ifelse(df_converged$Metric == "True Success", 1, 0)

# Step 1: Subset the data for the 1s condition
# Note: Adjust "1s Duration" if your string looks exactly like "1s" or "1s Duration"
df_1s <- subset(df_converged, Duration == "1s Duration" | Duration == "1s")

# Step 2: Run the Weighted Binomial GLM
model_1s <- glm(Is_True_Success ~ Algorithm, 
                data = df_1s, 
                weights = Count, 
                family = binomial,
                method = "brglmFit")

# Step 3: Extract the full pairwise comparisons matrix
pairs_1s <- emmeans(model_1s, pairwise ~ Algorithm, type = "response")

# View the results
pairs_1s$emmeans   # Shows predicted success probabilities per algorithm
pairs_1s$contrasts # Shows the pairwise comparisons (Odds Ratios and p-values)

# Step 1: Subset the data for the 5s condition
df_5s <- subset(df_converged, Duration == "5s Duration" | Duration == "5s")

# Step 2: Run the Weighted Binomial GLM
model_5s <- glm(Is_True_Success ~ Algorithm, 
                data = df_5s, 
                weights = Count, 
                family = binomial,
                method = "brglmFit")

# Step 3: Extract the full pairwise comparisons matrix
pairs_5s <- emmeans(model_5s, pairwise ~ Algorithm, type = "response")

# View the results
pairs_5s$emmeans
pairs_5s$contrasts






