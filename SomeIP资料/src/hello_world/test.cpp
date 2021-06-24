#include<stdio.h>
#include<math.h>
int main()
{
    int start=300, end=1000, i, k, m, flag=1, h=0;

    while(!(start>0 && start<end));
    printf("......... prime table(%d-%d).........\n", start, end);
    for(m=start; m<=end; m++)
    {
        k=sqrt(m);
        for(i=2; i<=k; i++)
            if(m%i==0)
            {
                flag=0;
                break;
            }
        if(flag)
        {
            printf("%-4d",m);
            h++;
            if(h%10==0)
                printf("\n");
        }
        flag=1; 
    }
    printf("\nThe total is %d", h);
    return 0;
}