# SmartCar
2020年第十五届全国大学生智能汽车竞赛-双车组三轮图像处理

主要处理思路，见[博客](https://blog.csdn.net/LeviKing98/article/details/107902795?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522167065575916782395379574%2522%252C%2522scm%2522%253A%252220140713.130102334.pc%255Fall.%2522%257D&request_id=167065575916782395379574&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~first_rank_ecpm_v1~rank_v31_ecpm-1-107902795-null-null.142^v68^control,201^v4^add_ask,213^v2^t3_control2&utm_term=%E7%AC%AC%E5%8D%81%E4%BA%94%E5%B1%8A%E5%85%A8%E5%9B%BD%E5%A4%A7%E5%AD%A6%E7%94%9F%E6%99%BA%E8%83%BD%E6%B1%BD%E8%BD%A6%E7%AB%9E%E8%B5%9B-%E5%8F%8C%E8%BD%A6%E7%BB%84%E4%B8%89%E8%BD%AE%E5%9B%BE%E5%83%8F%E5%A4%84%E7%90%86%E6%80%BB%E7%BB%93%EF%BC%88%E5%B7%B2%E5%BC%80%E6%BA%90%EF%BC%89&spm=1018.2226.3001.4187)  

直角型拐点、圆弧形拐点：见博客  
第一路段、第二路段：赛道被分割后呈现的情况，离车辆近的为第一路段，离车辆远的为第二路段，如十字。  
领域边线：八邻域生长出的赛道轮廓  
赛道边线：通过**邻域边线**提取的边线（每行仅有左右边线各一个点），用来计算中线
