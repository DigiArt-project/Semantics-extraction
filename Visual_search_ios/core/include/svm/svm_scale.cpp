

#include "svm_scale.hpp"


char *line = NULL;
int max_line_len = 1024;
double lower=-1.0,upper=1.0,y_lower,y_upper;
int y_scaling = 0;
double *feature_max;
double *feature_min;
double y_max = -DBL_MAX;
double y_min = DBL_MAX;
int max_index;
int min_index;
long int num_nonzeros = 0;
long int new_num_nonzeros = 0;
int count = 0;
std::string name_filename_result;


char* readline_scale(FILE *input)
{
    int len;
    
    if(fgets(line,max_line_len,input) == NULL)
        return NULL;
    
    while(strrchr(line,'\n') == NULL)
    {
        max_line_len *= 2;
        line = (char *) realloc(line, max_line_len);
        len = (int) strlen(line);
        if(fgets(line+len,max_line_len-len,input) == NULL)
            break;
    }
    return line;
}

void output_target(double value)
{
    std::ofstream file;
    file.open(name_filename_result, std::ios::out | std::ios::app );
    if(y_scaling)
    {
        if(value == y_min)
            value = y_lower;
        else if(value == y_max)
            value = y_upper;
        else value = y_lower + (y_upper-y_lower) *
            (value - y_min)/(y_max-y_min);
    }
    if (count == 0){
        std::string out =std::to_string(value);
        file << out;
    }else {
        std::string out = "\n" + std::to_string(value);
        file << out;
    }

    
   
    //std::cout << count << std::endl;
     count ++;
    
    
    //printf("%g ",value);
}

void output(int index, double value)
{
    std::ofstream file;
    file.open(name_filename_result, std::ios::out | std::ios::app );
    /* skip single-valued attribute */
    if(feature_max[index] == feature_min[index])
        return;
    
    if(value == feature_min[index])
        value = lower;
    else if(value == feature_max[index])
        value = upper;
    else
        value = lower + (upper-lower) *
        (value-feature_min[index])/
        (feature_max[index]-feature_min[index]);
    
    if(value != 0)
    {
        std::string out = " "+std::to_string(index)+":" +std::to_string(value);
        file << out;
        
        
        //file << printf("%d:%g ",index, value);
        new_num_nonzeros++;
   
    }
    
}

int clean_up(FILE *fp_restore, FILE *fp, const char* msg)
{
    fprintf(stderr,	"%s", msg);
    free(line);
    free(feature_max);
    free(feature_min);
    fclose(fp);
    if (fp_restore)
        fclose(fp_restore);
    return -1;
}



bool scale(const std::string data_filename,const std::string path_to_saveScaleFileResult,const std::string path_to_saveRangeFile, bool restore, double lower, double upper, bool scaling_limits){
    
    
    std::cout << "[INFO] Going to scale : "<< data_filename << " between " << std::to_string(lower) << " and " << std::to_string(upper) << std::endl;
    std::cout << "[INFO] scaling limites : "<< scaling_limits << std::endl;
    std::cout << "[INFO] Please wait... " << std::flush;
    int i,index;
    FILE *fp, *fp_restore = NULL;
    std::string save_filename_scale = path_to_saveRangeFile.c_str();
    std::string save_filename_result = path_to_saveScaleFileResult.c_str();
    name_filename_result = save_filename_result;
    std::string restore_filename;
    if (restore){
        restore_filename = path_to_saveRangeFile;
    }
    
    double y_scaling = 0;

    
    if (scaling_limits){
        y_scaling = 1;
    }
    
    
    if(!(upper > lower) || (y_scaling && !(y_upper > y_lower)))
    {
        fprintf(stderr,"inconsistent lower/upper specification\n");
        exit(1);
    }
    
    fp=fopen(data_filename.c_str(),"r");
    
    if(fp==NULL)
    {
        fprintf(stderr,"can't open file %s\n", data_filename.c_str());
        exit(1);
    }
    
    line = (char *) malloc(max_line_len*sizeof(char));
    
#define SKIP_TARGET\
    while(isspace(*p)) ++p;\
    while(!isspace(*p)) ++p;
    
#define SKIP_ELEMENT\
    while(*p!=':') ++p;\
    ++p;\
    while(isspace(*p)) ++p;\
    while(*p && !isspace(*p)) ++p;
    
    /* assumption: min index of attributes is 1 */
    /* pass 1: find out max index of attributes */
    max_index = 0;
    min_index = 1;
    
    if(restore)
    {
        std::cout << "INFO : RESTORE " << std::endl;
        int idx, c;
        
        fp_restore = fopen(restore_filename.c_str(),"r");
        if(fp_restore==NULL)
        {
            fprintf(stderr,"can't open file %s\n", restore_filename.c_str());
            exit(1);
        }
        
        c = fgetc(fp_restore);
        if(c == 'y')
        {
            readline_scale(fp_restore);
            readline_scale(fp_restore);
            readline_scale(fp_restore);
        }
        readline_scale(fp_restore);
        readline_scale(fp_restore);
        
        while(fscanf(fp_restore,"%d %*f %*f\n",&idx) == 1)
            max_index = max(idx,max_index);
        rewind(fp_restore);
    }
    
    while(readline_scale(fp)!=NULL)
    {
        char *p=line;
        
        SKIP_TARGET
        
        while(sscanf(p,"%d:%*f",&index)==1)
        {
            max_index = max(max_index, index);
            min_index = min(min_index, index);
            SKIP_ELEMENT
            num_nonzeros++;
        }
    }
    
    if(min_index < 1)
        fprintf(stderr,
                "WARNING: minimal feature index is %d, but indices should start from 1\n", min_index);
    
    rewind(fp);
    
    feature_max = (double *)malloc((max_index+1)* sizeof(double));
    feature_min = (double *)malloc((max_index+1)* sizeof(double));
    
    if(feature_max == NULL || feature_min == NULL)
    {
        fprintf(stderr,"can't allocate enough memory\n");
        exit(1);
    }
    
    for(i=0;i<=max_index;i++)
    {
        feature_max[i]=-DBL_MAX;
        feature_min[i]=DBL_MAX;
    }
    
    /* pass 2: find out min/max value */
    while(readline_scale(fp)!=NULL)
    {
        char *p=line;
        int next_index=1;
        double target;
        double value;
        
        if (sscanf(p,"%lf",&target) != 1)
            return clean_up(fp_restore, fp, "ERROR: failed to read labels\n");
        y_max = max(y_max,target);
        y_min = min(y_min,target);
        
        SKIP_TARGET
        
        while(sscanf(p,"%d:%lf",&index,&value)==2)
        {
            for(i=next_index;i<index;i++)
            {
                feature_max[i]=max(feature_max[i],0);
                feature_min[i]=min(feature_min[i],0);
            }
            
            feature_max[index]=max(feature_max[index],value);
            feature_min[index]=min(feature_min[index],value);
            
            SKIP_ELEMENT
            next_index=index+1;
        }
        
        for(i=next_index;i<=max_index;i++)
        {
            feature_max[i]=max(feature_max[i],0);
            feature_min[i]=min(feature_min[i],0);
        }
    }
    
    rewind(fp);
    
    /* pass 2.5: save/restore feature_min/feature_max */
    
    if(restore)
    {
        /* fp_restore rewinded in finding max_index */
        int idx, c;
        double fmin, fmax;
        int next_index = 1;
        
        if((c = fgetc(fp_restore)) == 'y')
        {
            if(fscanf(fp_restore, "%lf %lf\n", &y_lower, &y_upper) != 2 ||
               fscanf(fp_restore, "%lf %lf\n", &y_min, &y_max) != 2)
                return clean_up(fp_restore, fp, "ERROR: failed to read scaling parameters\n");
            y_scaling = 1;
        }
        else
            ungetc(c, fp_restore);
        
        if (fgetc(fp_restore) == 'x')
        {
            if(fscanf(fp_restore, "%lf %lf\n", &lower, &upper) != 2)
                return clean_up(fp_restore, fp, "ERROR: failed to read scaling parameters\n");
            while(fscanf(fp_restore,"%d %lf %lf\n",&idx,&fmin,&fmax)==3)
            {
                for(i = next_index;i<idx;i++)
                    if(feature_min[i] != feature_max[i])
                        fprintf(stderr,
                                "WARNING: feature index %d appeared in file %s was not seen in the scaling factor file %s.\n",
                                i, data_filename.c_str(), restore_filename.c_str());
                
                feature_min[idx] = fmin;
                feature_max[idx] = fmax;
                
                next_index = idx + 1;
            }
            
            for(i=next_index;i<=max_index;i++)
                if(feature_min[i] != feature_max[i])
                    fprintf(stderr,
                            "WARNING: feature index %d appeared in file %s was not seen in the scaling factor file %s.\n",
                            i, data_filename.c_str(), restore_filename.c_str());
        }
        fclose(fp_restore);
    }
    
    if (!restore){
        
    
    
    if(save_filename_scale.c_str())
    {
        std::cout << "Save filename scale " << std::endl;
        FILE *fp_save = fopen(save_filename_scale.c_str(),"w");
        if(fp_save==NULL)
        {
            fprintf(stderr,"can't open file %s\n", save_filename_scale.c_str());
            exit(1);
        }
        if(y_scaling)
        {
            fprintf(fp_save, "y\n");
            fprintf(fp_save, "%.16g %.16g\n", y_lower, y_upper);
            fprintf(fp_save, "%.16g %.16g\n", y_min, y_max);
        }
        fprintf(fp_save, "x\n");
        fprintf(fp_save, "%.16g %.16g\n", lower, upper);
        for(i=1;i<=max_index;i++)
        {
            if(feature_min[i]!=feature_max[i])
                fprintf(fp_save,"%d %.16g %.16g\n",i,feature_min[i],feature_max[i]);
        }
        
        if(min_index < 1)
            fprintf(stderr,
                    "WARNING: scaling factors with indices smaller than 1 are not stored to the file %s.\n", save_filename_scale.c_str());
        
        fclose(fp_save);
    }
    }
    /* pass 3: scale */
    while(readline_scale(fp)!=NULL)
    {
        char *p=line;
        int next_index=1;
        double target;
        double value;
        
        if (sscanf(p,"%lf",&target) != 1)
            return clean_up(NULL, fp, "ERROR: failed to read labels\n");
        output_target(target);
        
        SKIP_TARGET
        
        while(sscanf(p,"%d:%lf",&index,&value)==2)
        {
            for(i=next_index;i<index;i++)
                output(i,0);
            
            output(index,value);
            
            SKIP_ELEMENT
            next_index=index+1;
        }
        
        for(i=next_index;i<=max_index;i++)
            output(i,0);
        
        //printf("\n");
    }
    
    if (new_num_nonzeros > num_nonzeros)
        fprintf(stderr,
                "WARNING: original #nonzeros %ld\n"
                "       > new      #nonzeros %ld\n"
                "If feature values are non-negative and sparse, use -l 0 rather than the default -l -1\n",
                num_nonzeros, new_num_nonzeros);
    
    free(line);
    free(feature_max);
    free(feature_min);
    fclose(fp);
    std::ofstream file;
    file.open(save_filename_result, std::ios::out | std::ios::app );
    file << "\n";
    std::cout << "Completed " << std::endl;
    return 0;
    
}
